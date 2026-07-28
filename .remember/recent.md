# Recent Milestones
- Phase 5 (Transport cutover to Ethernet) completed.
- Implemented robust Ethernet hot-plug recovery, ensuring that cable replugs gracefully restart DHCP and mDNS without firmware hard-faults.
- Host logic updated to actively resolve mDNS rather than failing back to broadcasts that don't route.
