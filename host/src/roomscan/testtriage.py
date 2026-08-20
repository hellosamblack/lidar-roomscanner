"""Static import-graph test triage: map a git diff to the likely-impacted tests.

Deterministic and DB-free (unlike coverage-based selectors such as testmon), so it
works cold in a fresh fleet worktree with no warm-up run. It leans on the repo's
clean ``test_<module>.py`` convention: it AST-parses every internal module + test
under ``host/`` to build a source->test dependency graph, then intersects a test's
transitive import closure with the set of changed modules.

Correctness-first. A **false negative** -- skipping a test that a change actually
broke -- is the failure this repo cares most about, so every case the graph cannot
map with confidence forces a **full run** rather than an empty selection:

  * a foundational file (``protocol.py``, ``config.py``, ``pyproject.toml`` ...),
  * a changed test *helper* (shared fixtures/golden code),
  * a changed ``.py`` under the graph's roots that resolves to no known module.

A handful of cross-cutting tests scan the tree/docs instead of importing one module,
so the graph is blind to them; those are in ``ALWAYS_RUN`` and are always selected.
Non-Python producers (the C transform shim, the Pi-bridge image) map to their tests
via ``PATH_TEST_RULES``.

This is the local/agent fast path only. **CI still runs the whole suite** -- triage
is never the gate that lets a regression onto ``main``.
"""
from __future__ import annotations

import ast
import subprocess
from functools import lru_cache
from pathlib import Path

# host/src/roomscan/testtriage.py -> host/
HOST = Path(__file__).resolve().parents[2]
REPO = HOST.parent
SRC_ROOT = HOST / "src"          # roomscan.* modules live under here
ROOMSCAN = SRC_ROOT / "roomscan"
TOOLS = HOST / "tools"
TESTS = HOST / "tests"

# --- conservative guardrails -------------------------------------------------

# Changing any of these fans out across the whole suite (wire format, runtime
# config, the test harness itself); cheaper and safer to just run everything.
FULL_RUN_TRIGGERS = frozenset({
    "host/src/roomscan/protocol.py",
    "host/src/roomscan/config.py",
    "host/src/roomscan/mcp_server/paths.py",
    "host/pyproject.toml",
    "host/tests/conftest.py",
})
# Any change under these dirs -> full run (golden wire vectors are shared).
FULL_RUN_DIR_TRIGGERS = ("host/tests/fixtures/",)

# Cross-cutting tests the import graph cannot see (they glob the tree / docs
# rather than importing a single module). Always selected.
ALWAYS_RUN = (
    "test_static_ui.py",
    "test_doc_links.py",
    "test_mcp_registry.py",
    "test_equivalence.py",
)

# Non-Python producers -> the tests that build/exercise them. Prefix match on the
# repo-relative posix path.
PATH_TEST_RULES = (
    ("host/transform/", ("test_protocol_c_crosscheck.py",)),
    ("pi-bridge/", ("test_pi_bridge_build.py", "test_pi_bridge_tools.py")),
)

# Changes here are covered by an ALWAYS_RUN test (static assets -> test_static_ui),
# so they need neither a module mapping nor a full run.
COVERED_BY_ALWAYS_RUN_DIRS = ("host/src/roomscan/static/",)


# --- import graph ------------------------------------------------------------

def _module_files() -> dict[Path, str]:
    """Map every internal ``.py`` (roomscan.* and tools.*) to its dotted name."""
    out: dict[Path, str] = {}
    for p in ROOMSCAN.rglob("*.py"):
        if "__pycache__" in p.parts:
            continue
        rel = p.relative_to(SRC_ROOT).with_suffix("")
        parts = [x for x in rel.parts if x != "__init__"]
        out[p] = ".".join(parts)
    if TOOLS.is_dir():
        for p in TOOLS.rglob("*.py"):
            if "__pycache__" in p.parts:
                continue
            rel = p.relative_to(HOST).with_suffix("")
            parts = [x for x in rel.parts if x != "__init__"]
            out[p] = ".".join(parts)
    return out


def _resolve(name: str, modules: set[str]) -> str | None:
    """Longest known-module prefix of a dotted ``name`` (or None)."""
    while name:
        if name in modules:
            return name
        if "." not in name:
            return None
        name = name.rsplit(".", 1)[0]
    return None


def _imports_of(path: Path, own_dotted: str, modules: set[str]) -> set[str]:
    """Internal modules directly imported by ``path``."""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (SyntaxError, UnicodeDecodeError, OSError):
        return set()
    is_pkg = path.name == "__init__.py"
    own_parts = own_dotted.split(".") if own_dotted else []
    base_parts = own_parts if is_pkg else own_parts[:-1]
    targets: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                r = _resolve(alias.name, modules)
                if r:
                    targets.add(r)
        elif isinstance(node, ast.ImportFrom):
            if node.level:                       # relative import
                up = node.level - 1
                anchor = base_parts[:-up] if up else base_parts
                if up and up > len(base_parts):
                    anchor = []
                mod_parts = anchor + (node.module.split(".") if node.module else [])
            else:
                mod_parts = node.module.split(".") if node.module else []
            full = ".".join(mod_parts)
            r = _resolve(full, modules)
            if r:
                targets.add(r)
            for alias in node.names:
                r = _resolve(f"{full}.{alias.name}".lstrip("."), modules)
                if r:
                    targets.add(r)
    return targets


@lru_cache(maxsize=1)
def build_import_graph() -> dict:
    """Build the module dependency graph and each test's transitive closure.

    Returns ``{"deps": {mod: {mod,...}}, "test_closure": {test_file: {mod,...}}}``
    where ``test_file`` is the ``tests/test_x.py`` posix path relative to ``host/``.
    """
    mod_files = _module_files()
    modules = set(mod_files.values())
    deps: dict[str, set[str]] = {}
    for path, dotted in mod_files.items():
        deps[dotted] = _imports_of(path, dotted, modules)

    def closure(start: set[str]) -> set[str]:
        seen: set[str] = set()
        stack = list(start)
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            seen.add(x)
            stack.extend(deps.get(x, ()))
        return seen

    test_closure: dict[str, set[str]] = {}
    for tp in sorted(TESTS.glob("test_*.py")):
        direct = _imports_of(tp, f"tests.{tp.stem}", modules)
        test_closure[f"tests/{tp.name}"] = closure(direct)
    return {"deps": deps, "test_closure": test_closure}


# --- change classification + selection ---------------------------------------

def _git(args: list[str]) -> str:
    try:
        p = subprocess.run(["git", *args], cwd=str(REPO), capture_output=True,
                           text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return ""
    return p.stdout if p.returncode == 0 else ""


def _changed_files(base: str) -> tuple[list[str], str]:
    """Repo-relative posix paths changed vs the merge-base with ``base``, plus
    the resolved merge-base ref (empty if it could not be resolved)."""
    mb = _git(["merge-base", base, "HEAD"]).strip()
    diff_ref = mb or "HEAD"
    changed = set(_git(["diff", "--name-only", diff_ref]).split())
    changed |= set(_git(["ls-files", "--others", "--exclude-standard"]).split())
    return sorted(changed), mb


def _path_to_module(path: str, modules: set[str]) -> str | None:
    """Map a repo-relative ``host/src/roomscan|tools/....py`` path to its module."""
    p = Path(path)
    if path.startswith("host/src/roomscan/"):
        dotted = ".".join(p.relative_to("host/src").with_suffix("").parts)
    elif path.startswith("host/tools/"):
        dotted = ".".join(p.relative_to("host").with_suffix("").parts)
    else:
        return None
    dotted = dotted.replace(".__init__", "")
    return dotted if dotted in modules else None


def impacted_tests(base: str = "origin/main", changed: list[str] | None = None) -> dict:
    """Select the tests likely impacted by the working tree's diff vs ``base``.

    Pass ``changed`` (repo-relative posix paths) to bypass git discovery -- used by
    the unit tests. Returns a dict with ``selected`` (``tests/test_x.py`` paths
    relative to ``host/``), per-test ``reasons``, ``full_run``, ``changed`` and
    ``base``. On ``full_run`` the whole suite is selected.
    """
    graph = build_import_graph()
    modules = set(graph["deps"])
    all_tests = sorted(graph["test_closure"])
    if changed is None:
        changed, mb = _changed_files(base)
    else:
        changed, mb = sorted(changed), ""

    full_run = False
    full_reason = ""
    changed_modules: dict[str, str] = {}   # module -> changed path (for reasons)
    reasons: dict[str, str] = {}

    def force_full(why: str) -> None:
        nonlocal full_run, full_reason
        full_run = True
        if not full_reason:
            full_reason = why

    for f in changed:
        if f in FULL_RUN_TRIGGERS:
            force_full(f"foundational file changed: {f}")
            continue
        if f.startswith(FULL_RUN_DIR_TRIGGERS):
            force_full(f"shared fixtures changed: {f}")
            continue
        rule_hit = False
        for prefix, tests in PATH_TEST_RULES:
            if f.startswith(prefix):
                for t in tests:
                    reasons.setdefault(f"tests/{t}", f"path-rule: {prefix} changed")
                rule_hit = True
        if rule_hit:
            continue
        if f.startswith("host/tests/"):
            name = Path(f).name
            if name.startswith("test_") and name.endswith(".py"):
                reasons[f"tests/{name}"] = "changed test file"
            else:
                force_full(f"shared test helper changed: {f}")
            continue
        if f.startswith(COVERED_BY_ALWAYS_RUN_DIRS):
            continue                                     # test_static_ui covers it
        if f.startswith(("host/src/roomscan/", "host/tools/")):
            if f.endswith(".py"):
                mod = _path_to_module(f, modules)
                if mod:
                    changed_modules.setdefault(mod, f)
                else:
                    force_full(f"unmapped source file changed: {f}")
            else:
                force_full(f"non-python source change: {f}")
            continue
        # Anything else (firmware/, docs/, .claude/, repo-root) has no host-test
        # impact the graph must reason about -- docs are covered by test_doc_links
        # (ALWAYS_RUN), firmware by CI's own job.

    for t in ALWAYS_RUN:
        reasons.setdefault(f"tests/{t}", "always-run (cross-cutting)")

    if full_run:
        selected = all_tests
        sel_reasons = {t: full_reason for t in selected}
        return {
            "base": base, "merge_base": mb, "changed": changed,
            "full_run": True, "full_run_reason": full_reason,
            "selected": selected, "reasons": sel_reasons,
            "selected_count": len(selected), "total_tests": len(all_tests),
        }

    if changed_modules:
        for test, closure in graph["test_closure"].items():
            hit = closure & set(changed_modules)
            if hit:
                mod = sorted(hit)[0]
                reasons.setdefault(
                    test, f"imports {mod} (changed via {changed_modules[mod]})")

    selected = sorted(t for t in reasons if t in graph["test_closure"])
    return {
        "base": base, "merge_base": mb, "changed": changed,
        "full_run": False, "full_run_reason": "",
        "selected": selected, "reasons": {t: reasons[t] for t in selected},
        "selected_count": len(selected), "total_tests": len(all_tests),
    }


def _main(argv: list[str] | None = None) -> int:
    import argparse
    import json
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--base", default="origin/main", help="git ref to diff against")
    ap.add_argument("--json", action="store_true", help="emit the full dict as JSON")
    args = ap.parse_args(argv)
    r = impacted_tests(base=args.base)
    if args.json:
        print(json.dumps(r, indent=2))
        return 0
    if r["full_run"]:
        print(f"FULL RUN ({r['total_tests']} tests): {r['full_run_reason']}")
    else:
        print(f"{r['selected_count']}/{r['total_tests']} tests impacted "
              f"({len(r['changed'])} files changed vs {r['base']}):")
        for t in r["selected"]:
            print(f"  {t}\t{r['reasons'][t]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
