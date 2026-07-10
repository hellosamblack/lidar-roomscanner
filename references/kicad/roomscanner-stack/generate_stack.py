#!/usr/bin/env python3
"""Generate the consolidated NUCLEO stack KiCad project (roomscanner-stack).

Stack (bottom -> top):
  NUCLEO-H563ZI (base) -> X-NUCLEO-IKS4A1 (middle) -> X-NUCLEO-53L9A1 (top)

One flat schematic, three titled board sections. Nets common across the stack
(the ST Zio / Arduino Uno V3 headers) are GLOBAL labels named by their STM32 pin;
board-internal nets are LOCAL labels. Same-named labels merge -> the netlist
proves the shared bus. Datasheets are copied into datasheets/ and linked via each
symbol's Datasheet field.

Reference numbering: base keeps real silk names; middle board +100, top +200
(last digits preserve the real name, e.g. CN105 = CN5, U107 = U7).
"""
import uuid, os, hashlib, shutil
from kiutils.symbol import Symbol, SymbolLib, SymbolPin
from kiutils.items.common import Position, Effects, Font, Property, Stroke, Fill, Justify
from kiutils.schematic import Schematic
from kiutils.items.schitems import (SchematicSymbol, GlobalLabel, LocalLabel,
    NoConnect, Text, Rectangle, SymbolProjectInstance, SymbolProjectPath)
from kiutils.items.syitems import SyRect

OUT  = r"F:/git/personal/lidar/roomscanner/references/kicad/roomscanner-stack"
DSRC = r"F:/git/personal/lidar/roomscanner/references/datasheets"
PROJ = "roomscanner-stack"
NICK = "roomscanner"
os.makedirs(OUT, exist_ok=True)

_ctr = [0]
def uid(seed=None):
    if seed is None:
        _ctr[0] += 1; seed = f"auto{_ctr[0]}"
    return str(uuid.UUID(hashlib.md5(("rmscan:"+seed).encode()).hexdigest()))
ROOT_UUID = uid("root")
def snap(v): return round(v/1.27)*1.27      # KiCad 50-mil connection grid

# ----------------------------------------------------------------------------
# Copy datasheets + application notes into the project (git-trackable)
# ----------------------------------------------------------------------------
DSDIR = os.path.join(OUT, "datasheets")
ANDIR = os.path.join(DSDIR, "appnotes")
os.makedirs(ANDIR, exist_ok=True)
# (dest, source-rel, provenance-note)
DS_COPY = [
 ("STM32H563_datasheet.pdf",     "NUCLEO-H563ZI/chipDatasheet.pdf",              "STM32H563 MCU datasheet"),
 ("NUCLEO-H563ZI_board.pdf",     "NUCLEO-H563ZI/x-nucleo-datasheet.pdf",         "NUCLEO-H563ZI board document"),
 ("X-NUCLEO-IKS4A1_board.pdf",   "NUCLEO-IKS4A1/datasheet.pdf",                  "X-NUCLEO-IKS4A1 board datasheet"),
 ("X-NUCLEO-53L9A1_board.pdf",   "NUCLEO-VL53L9CX/x-nucleo-datasheet.pdf",       "X-NUCLEO-53L9A1 board datasheet"),
 ("VL53L9CX_datasheet.pdf",      "NUCLEO-VL53L9CX/datasheet.pdf",                "VL53L9CX ToF device datasheet"),
 ("LSM6DSV16X_datasheet.pdf",    "NUCLEO-IKS4A1/LSM6DSV16XTR/datasheet.pdf",     "LSM6DSV16X IMU datasheet"),
 ("X-NUCLEO-53L9A1_schematic.pdf","NUCLEO-VL53L9CX/schematic/x-nucleo-53l9a1-schematic.pdf","53L9A1 board schematic"),
 ("X-NUCLEO-IKS4A1_schematic.pdf","NUCLEO-IKS4A1/schematic/schematic.pdf",       "IKS4A1 board schematic"),
]
AN_COPY = [
 ("LSM6DSV16X_appnote.pdf",      "NUCLEO-IKS4A1/LSM6DSV16XTR/applicationNote.pdf",   "LSM6DSV16X application note"),
 ("LSM6DSV16X_MLC.pdf",          "NUCLEO-IKS4A1/LSM6DSV16XTR/machineLearningCore.pdf","LSM6DSV16X machine-learning-core"),
 ("DT0058_tilt_ecompass.pdf",    "NUCLEO-IKS4A1/LSM6DSV16XTR/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf","design tip: tilt/e-compass"),
 ("DT0060_gyro_tilt.pdf",        "NUCLEO-IKS4A1/LSM6DSV16XTR/dt0060-exploiting-the-gyroscope-to-update-tilt-measurement-and-ecompass-stmicroelectronics.pdf","design tip: gyro tilt"),
 ("DT0064_noise_analysis.pdf",   "NUCLEO-IKS4A1/LSM6DSV16XTR/dt0064-noise-analysis-and-identification-in-mems-sensors-allan-time-hadamard-overlapping-modified-total-variance-stmicroelectronics.pdf","design tip: noise analysis"),
 ("DT0106_deadreckoning.pdf",    "NUCLEO-IKS4A1/LSM6DSV16XTR/dt0106-residual-linear-acceleration-by-gravity-subtraction-to-enable-deadreckoning-stmicroelectronics.pdf","design tip: dead-reckoning"),
 ("X-NUCLEO-IKS4A1_getting_started.pdf","NUCLEO-IKS4A1/gettingStarted.pdf",      "IKS4A1 getting started"),
 ("X-NUCLEO-IKS4A1_quick_start.pdf",    "NUCLEO-IKS4A1/quickStartGuide.pdf",     "IKS4A1 quick start guide"),
]
manifest = ["# Datasheets & application notes\n",
    "Copied into the project so the KiCad design is self-contained and git-trackable.",
    "Each symbol's **Datasheet** field points at the file here (or an ST/vendor URL",
    "for parts whose PDF we do not hold locally).\n",
    "| File | Source (references/datasheets/...) | Description |",
    "|------|-----------------------------------|-------------|"]
def do_copy(lst, subdir):
    for dest, src, desc in lst:
        s = os.path.join(DSRC, src)
        d = os.path.join(subdir, dest)
        if os.path.exists(s):
            shutil.copy2(s, d)
            rel = os.path.relpath(d, OUT).replace("\\","/")
            manifest.append(f"| `{rel}` | `{src}` | {desc} |")
        else:
            manifest.append(f"| (missing) | `{src}` | {desc} — SOURCE NOT FOUND |")
do_copy(DS_COPY, DSDIR)
do_copy(AN_COPY, ANDIR)

# Datasheet field targets (relative path if local, else vendor URL)
DS = {
  "board_h563": "datasheets/NUCLEO-H563ZI_board.pdf",
  "board_iks":  "datasheets/X-NUCLEO-IKS4A1_board.pdf",
  "board_vl53": "datasheets/X-NUCLEO-53L9A1_board.pdf",
  "VL53L9CX":   "datasheets/VL53L9CX_datasheet.pdf",
  "LSM6DSV16X": "datasheets/LSM6DSV16X_datasheet.pdf",
  "PI4ULS3V204":"https://www.diodes.com/assets/Datasheets/PI4ULS3V204.pdf",
  "NXS0108":    "https://www.nxp.com/docs/en/data-sheet/NXS0108.pdf",
  "LDK130":     "https://www.st.com/resource/en/datasheet/ldk130.pdf",
  "LIS2MDL":    "https://www.st.com/resource/en/datasheet/lis2mdl.pdf",
  "LPS22DF":    "https://www.st.com/resource/en/datasheet/lps22df.pdf",
  "LIS2DUXS12": "https://www.st.com/resource/en/datasheet/lis2duxs12.pdf",
  "STTS22H":    "https://www.st.com/resource/en/datasheet/stts22h.pdf",
  "LSM6DSO16IS":"https://www.st.com/resource/en/datasheet/lsm6dso16is.pdf",
  "SHT40":      "https://sensirion.com/products/catalog/SHT40-AD1B",
  "OSC":        "",
}

# ----------------------------------------------------------------------------
# Symbol library
# ----------------------------------------------------------------------------
libsyms = {}
def _pin(etype, x, y, ang, length, name, number):
    p = SymbolPin(electricalType=etype, graphicalStyle="line",
                  position=Position(x, y, ang), length=length, name=name, number=str(number))
    p.nameEffects = Effects(font=Font(height=1.0, width=1.0))
    p.numberEffects = Effects(font=Font(height=1.0, width=1.0))
    return p

def make_connector(entry, npins):
    s = Symbol.create_new(id=f"{NICK}:{entry}", reference="J", value=entry)
    s.pinNames = True; s.pinNamesHide = True
    u = Symbol(); u.libId = f"{entry}_1_1"
    top = (npins-1)*2.54/2.0
    for i in range(1, npins+1):
        u.pins.append(_pin("passive", 7.62, top-(i-1)*2.54, 180, 5.08, f"Pin_{i}", i))
    u.graphicItems.append(SyRect(start=Position(-2.54, top+2.54),
        end=Position(2.54, top-npins*2.54), stroke=Stroke(width=0.2032), fill=Fill(type="background")))
    s.units.append(u); libsyms[entry]=s

def make_connector2(entry, nrows):
    s = Symbol.create_new(id=f"{NICK}:{entry}", reference="J", value=entry)
    s.pinNames = True; s.pinNamesHide = True
    u = Symbol(); u.libId = f"{entry}_1_1"
    top = (nrows-1)*2.54/2.0
    for r in range(nrows):
        y = top-r*2.54
        u.pins.append(_pin("passive", -7.62, y, 0, 5.08, f"Pin_{2*r+1}", 2*r+1))
        u.pins.append(_pin("passive",  7.62, y, 180, 5.08, f"Pin_{2*r+2}", 2*r+2))
    u.graphicItems.append(SyRect(start=Position(-2.54, top+2.54),
        end=Position(2.54, top-nrows*2.54), stroke=Stroke(width=0.2032), fill=Fill(type="background")))
    s.units.append(u); libsyms[entry]=s

def make_ic(entry, left, right, w=25.4):
    s = Symbol.create_new(id=f"{NICK}:{entry}", reference="U", value=entry)
    s.pinNames = True; s.pinNamesHide = False
    u = Symbol(); u.libId = f"{entry}_1_1"
    n = max(len(left), len(right)); top=(n-1)*2.54/2.0; half=w/2.0
    for i,(num,name) in enumerate(left):
        u.pins.append(_pin("passive", -half-3.81, top-i*2.54, 0, 3.81, name, num))
    for i,(num,name) in enumerate(right):
        u.pins.append(_pin("passive", half+3.81, top-i*2.54, 180, 3.81, name, num))
    u.graphicItems.append(SyRect(start=Position(-half, top+2.54),
        end=Position(half, top-n*2.54), stroke=Stroke(width=0.2032), fill=Fill(type="background")))
    s.units.append(u); libsyms[entry]=s

make_connector("Conn_1x06", 6); make_connector("Conn_1x08", 8); make_connector("Conn_1x10", 10)
make_connector2("Conn_2x19", 19)
make_ic("PI4ULS3V204", [(1,"VCCA"),(2,"A1"),(3,"A2"),(4,"A3"),(5,"A4"),(8,"EN"),(7,"GND")],
                       [(14,"VCCB"),(13,"B1"),(12,"B2"),(11,"B3"),(10,"B4")])
make_ic("NXS0108", [(2,"VCCA"),(1,"A1"),(3,"A2"),(4,"A3"),(11,"GND"),(10,"OE")],
                   [(19,"VCCB"),(20,"B1"),(18,"B2"),(17,"B3")])
make_ic("VL53L9CX", [("A11","SCL"),("A12","SDA"),("A10","INTR"),("B12","XSHUT"),("D12","SYNC_IN"),("E11","AP_CLK")],
                    [("E8","IOVDD"),("E6","AVDD"),("C12","DVDD"),("B1","VBAT_LDD"),("A1","GND")], w=30.48)
make_ic("LSM6DSV16X", [(13,"SCL"),(14,"SDA"),(1,"SDO/SA0"),(4,"INT1"),(9,"INT2"),(12,"CS")],
                      [(8,"VDD"),(5,"VDD_IO"),(6,"GND")], w=25.4)
make_ic("LDK130", [(1,"VIN"),(3,"EN")], [(5,"VOUT"),(4,"ADJ"),(2,"GND")], w=17.78)
make_ic("OSC_12MHz", [(1,"OE"),(2,"GND")], [(3,"OUT"),(4,"VDD")], w=15.24)
make_ic("SW_SPDT", [(1,"INT"),(3,"EXT")], [(2,"COM")], w=12.7)   # 53L9A1 SW1 clock select
# six secondary IKS4A1 sensors
make_ic("LIS2MDL",   [(1,"SCL"),(4,"SDA"),(7,"INT/DRDY")], [(9,"VDD"),(10,"VDD_IO"),(6,"GND")], w=20.32)
make_ic("LPS22DF",   [(2,"SCL"),(4,"SDA"),(7,"INT_DRDY")], [(10,"VDD"),(1,"VDD_IO"),(9,"GND")], w=20.32)
make_ic("LIS2DUXS12",[(1,"SCL"),(4,"SDA"),(12,"INT1"),(11,"INT2")], [(9,"VDD"),(10,"VDD_IO"),(6,"GND")], w=20.32)
make_ic("STTS22H",   [(1,"SCL"),(6,"SDA"),(2,"AL/INT")], [(3,"VDD"),(4,"ADDR"),(5,"GND")], w=20.32)
make_ic("LSM6DSO16IS",[(13,"SCL"),(14,"SDA"),(4,"INT1"),(9,"INT2")], [(8,"VDD"),(5,"VDDIO"),(6,"GND")], w=22.86)
make_ic("SHT40",     [(2,"SCL"),(1,"SDA")], [(3,"VDD"),(4,"GND")], w=17.78)

# ----------------------------------------------------------------------------
# Schematic assembly
# ----------------------------------------------------------------------------
sch = Schematic().create_new()
sch.version = "20230121"; sch.generator = "eeschema"
sch.uuid = ROOT_UUID; sch.paper.paperSize = "A1"
for s in libsyms.values(): sch.libSymbols.append(s)

def place(entry, ref, x, y, refy_off=-12.0, datasheet="", realname=None):
    x, y = snap(x), snap(y)
    inst = SchematicSymbol()
    inst.libraryNickname=NICK; inst.entryName=entry
    inst.inBom=True; inst.onBoard=True
    inst.position=Position(x, y, 0); inst.unit=1; inst.uuid=uid(f"sym:{ref}")
    inst.properties.append(Property(key="Reference", value=ref, id=0,
        position=Position(x, y+refy_off, 0), effects=Effects(font=Font(height=1.27,width=1.27))))
    inst.properties.append(Property(key="Value", value=entry, id=1,
        position=Position(x, y+refy_off+2.2, 0), effects=Effects(font=Font(height=1.0,width=1.0))))
    inst.properties.append(Property(key="Footprint", value="", id=2,
        position=Position(x, y, 0), effects=Effects(font=Font(height=1.27,width=1.27), hide=True)))
    inst.properties.append(Property(key="Datasheet", value=datasheet, id=3,
        position=Position(x, y, 0), effects=Effects(font=Font(height=1.27,width=1.27), hide=True)))
    inst.instances.append(SymbolProjectInstance(name=PROJ,
        paths=[SymbolProjectPath(sheetInstancePath="/", reference=ref, unit=1)]))
    sch.schematicSymbols.append(inst)
    if realname:
        note(realname, x-8, y+refy_off-2, size=1.1)
    coords={}
    for p in libsyms[entry].units[0].pins:
        coords[p.number]=(snap(x+p.position.X), snap(y-p.position.Y))
    return coords

def glabel(name, x, y, to_right=True, shape="bidirectional", size=1.0):
    e=Effects(font=Font(height=size,width=size)); e.justify=Justify(horizontally="left" if to_right else "right")
    sch.globalLabels.append(GlobalLabel(text=name, shape=shape,
        position=Position(snap(x),snap(y),0 if to_right else 180), effects=e, uuid=uid()))
def llabel(name, x, y, to_right=True, size=1.0):
    e=Effects(font=Font(height=size,width=size)); e.justify=Justify(horizontally="left" if to_right else "right")
    sch.labels.append(LocalLabel(text=name, position=Position(snap(x),snap(y),0 if to_right else 180), effects=e, uuid=uid()))
def noconn(x, y): sch.noConnects.append(NoConnect(position=Position(snap(x),snap(y)), uuid=uid()))
def note(text, x, y, size=1.4):
    e=Effects(font=Font(height=size,width=size)); e.justify=Justify(horizontally="left")
    sch.texts.append(Text(text=text, position=Position(x,y,0), effects=e, uuid=uid()))
def box(x1,y1,x2,y2):
    sch.shapes.append(Rectangle(start=Position(x1,y1), end=Position(x2,y2),
        stroke=Stroke(width=0.3, type="dash"), fill=Fill(type="none"), uuid=uid()))

# canonical H563 Zio/Arduino pinout (ground truth: X-NUCLEO-53L9A1 schematic)
CN5=["PF3","PD15","PD14","PB5","PG9","PA5",None,None,"PB9","PB8"]
CN6=[None,"IOREF","NRST","+3V3","+5V","GND","GND","VIN"]
CN8=["PA6","PC0","PC3","PB1","PC2","PF11"]
CN9=["PB7","PB6","PG14","PE13","PE14","PE11","PE9","PG12"]

def emit_connector(entry, ref, netlist, x, y, ds="", realname=None):
    top=(len(netlist))*2.54/2.0
    coords=place(entry, ref, x, y, refy_off=-(top+4), datasheet=ds)
    if realname: note(realname, x-3, snap(y)-top-6.5, size=1.1)
    for i,net in enumerate(netlist, start=1):
        sx,sy=coords[str(i)]
        if net is None: noconn(sx,sy)
        else: glabel(net, sx, sy, to_right=True, size=1.0)
    return coords

# ======================= SECTION 1 : H563 base =======================
X0=60
note("Consolidated NUCLEO stack  -  H563ZI (base) + IKS4A1 (middle) + 53L9A1 (top)", 60, 8, size=3.5)
note("Shared ST Zio/Arduino headers CN5/CN6/CN8/CN9 -> global-label nets merge the three boards. Built for roomscanner.", 60, 12.5, size=1.5)
note("NUCLEO-H563ZI  (base board)", X0-20, 22, size=3.0)
note("STM32H563ZI - ST Zio / Arduino Uno V3 headers (stack bus source)", X0-20, 28, size=1.6)
box(X0-25, 16, X0+95, 325)
emit_connector("Conn_1x10","CN5", CN5, X0, 58, ds=DS["board_h563"])
emit_connector("Conn_1x08","CN6", CN6, X0, 112, ds=DS["board_h563"])
emit_connector("Conn_1x06","CN8", CN8, X0, 162, ds=DS["board_h563"])
emit_connector("Conn_1x08","CN9", CN9, X0, 206, ds=DS["board_h563"])
cn7 =place("Conn_2x19","CN7",  X0+55,132, refy_off=-52, datasheet=DS["board_h563"])
cn10=place("Conn_2x19","CN10", X0+55,236, refy_off=-52, datasheet=DS["board_h563"])
for cc in (cn7,cn10):
    for num,(sx,sy) in cc.items(): noconn(sx,sy)
note("CN7 / CN10 = STM32 morpho (2x19) pass-through to IKS4A1.", X0+40, 262, size=1.2)
note("GPIO/analog carried 1:1 up the stack (UM3115); not used by", X0+40, 265, size=1.2)
note("the ToF/IMU sensor stack -> no-connect here.", X0+40, 268, size=1.2)
note("Key STM32 functions on the Arduino headers:", X0-20, 285, size=1.4)
note("PB8=I3C1_SCL  PB9=I3C1_SDA  PB5=TIM3_CH2(CLK_IN)", X0-20, 289, size=1.3)
note("PB1=SYNC_IN  PB6=XSHUT  PB7=INTR(EXTI7)", X0-20, 292, size=1.3)
note("Default jumpers (board config, not on Zio pins):", X0-20, 305, size=1.4)
note("JP2=1-2 (USB/UCPD power)   JP4=1-2 (VDD_MCU=3V3)", X0-20, 309, size=1.2)
note("JP5=closed (IDD measure)   JP6=closed (ST-LINK VCP UART)", X0-20, 312, size=1.2)

# ======================= SECTION 2 : IKS4A1 (middle) =======================
X1=300
note("X-NUCLEO-IKS4A1  (middle board)", X1-20, 22, size=3.0)
note("MEMS + environmental. Taps I2C on PB8/PB9 as legacy-I2C targets.", X1-20, 28, size=1.6)
box(X1-25, 16, X1+220, 390)
emit_connector("Conn_1x10","CN105", CN5, X1, 58,  ds=DS["board_iks"], realname="= CN5")
emit_connector("Conn_1x08","CN106", CN6, X1, 112, ds=DS["board_iks"], realname="= CN6")
emit_connector("Conn_1x06","CN108", CN8, X1, 162, ds=DS["board_iks"], realname="= CN8")
emit_connector("Conn_1x08","CN109", CN9, X1, 206, ds=DS["board_iks"], realname="= CN9")
ik7 =place("Conn_2x19","CN107", X1+55,132, refy_off=-52, datasheet=DS["board_iks"])
ik10=place("Conn_2x19","CN110", X1+55,236, refy_off=-52, datasheet=DS["board_iks"])
for cc in (ik7,ik10):
    for num,(sx,sy) in cc.items(): noconn(sx,sy)

# LDO 3V3 -> 1V8 (sensor Vio rail)
ld=place("LDK130","U101", X1+40, 300, refy_off=-10, datasheet=DS["LDK130"], realname="= U1")
glabel("+3V3",*ld["1"],to_right=False); glabel("GND",*ld["2"],to_right=False); glabel("+3V3",*ld["3"],to_right=False)
glabel("+1V8",*ld["5"],to_right=True); glabel("+1V8",*ld["4"],to_right=True)
note("U1 LDK130: +3V3 -> +1V8 (sensor Vio; JP-selectable to 3V3)", X1+18, 315, size=1.2)

# NXS0108 host I2C (3V3) <-> sensor bus (1V8)
sh=place("NXS0108","U103", X1+120, 66, refy_off=-14, datasheet=DS["NXS0108"], realname="= U3")
glabel("+3V3",*sh["2"],to_right=False); glabel("PB8",*sh["1"],to_right=False); glabel("PB9",*sh["3"],to_right=False)
glabel("GND",*sh["11"],to_right=False); noconn(*sh["4"]); noconn(*sh["10"])
glabel("+1V8",*sh["19"],to_right=True); llabel("IKS_SCL",*sh["20"],to_right=True); llabel("IKS_SDA",*sh["18"],to_right=True); noconn(*sh["17"])
note("U3 NXS0108: host I2C (PB8/PB9, 3V3) <-> sensor I2C bus (1V8).", X1+80, 84, size=1.2)

# LSM6DSV16X (HUB1) - SFLP IMU used by firmware (stream 9)
im=place("LSM6DSV16X","U104", X1+120, 138, refy_off=-12, datasheet=DS["LSM6DSV16X"], realname="= U4")
llabel("IKS_SCL",*im["13"],to_right=False); llabel("IKS_SDA",*im["14"],to_right=False)
glabel("+1V8",*im["1"],to_right=False); noconn(*im["4"]); noconn(*im["9"]); glabel("+1V8",*im["12"],to_right=False)
glabel("+1V8",*im["8"],to_right=True); glabel("+1V8",*im["5"],to_right=True); glabel("GND",*im["6"],to_right=True)
note("U4 LSM6DSV16X (HUB1) 0x50/0x52: SFLP orientation IMU -> fw stream 9.", X1+80, 158, size=1.2)

# six secondary sensors on the internal 1V8 I2C bus
SENSORS=[
 ("LIS2MDL","U107","= U7 (mag 0x1E)",   dict(scl=1,sda=4,vdd=9,vddio=10,gnd=6,ints=[7]),                X1+120,190),
 ("LPS22DF","U106","= U6 (baro 0x5D)",  dict(scl=2,sda=4,vdd=10,vddio=1,gnd=9,ints=[7]),                X1+185,190),
 ("LIS2DUXS12","U105","= U5 (accel 0x18)",dict(scl=1,sda=4,vdd=9,vddio=10,gnd=6,ints=[12,11]),         X1+120,238),
 ("STTS22H","U108","= U8 (temp 0x38)",  dict(scl=1,sda=6,vdd=3,vddio=None,gnd=5,ints=[2],addr=4),       X1+185,238),
 ("LSM6DSO16IS","U109","= U9 (HUB2 0x6A)",dict(scl=13,sda=14,vdd=8,vddio=5,gnd=6,ints=[4,9]),           X1+120,288),
 ("SHT40","U110","= U10 (RH/T 0x44)",   dict(scl=2,sda=1,vdd=3,vddio=None,gnd=4,ints=[]),               X1+185,288),
]
for entry,ref,real,pn,sx0,sy0 in SENSORS:
    c=place(entry, ref, sx0, sy0, refy_off=-11, datasheet=DS[entry], realname=real)
    llabel("IKS_SCL",*c[str(pn["scl"])],to_right=False); llabel("IKS_SDA",*c[str(pn["sda"])],to_right=False)
    glabel("+1V8",*c[str(pn["vdd"])],to_right=True)
    if pn["vddio"]: glabel("+1V8",*c[str(pn["vddio"])],to_right=True)
    glabel("GND",*c[str(pn["gnd"])],to_right=True)
    if pn.get("addr"): glabel("GND",*c[str(pn["addr"])],to_right=True)   # STTS22H ADDR->GND
    for ip in pn["ints"]: noconn(*c[str(ip)])
note("Six secondary sensors above share the internal 1V8 I2C bus (IKS_SCL/IKS_SDA).", X1+95, 320, size=1.3)
note("Real board splits it into STM_I2C / SENS_I2C / HUB2_I2C sub-buses via solder", X1+95, 323, size=1.1)
note("bridges, and HUB1/HUB2 can master sensors over their aux I2C. INT/DRDY lines", X1+95, 326, size=1.1)
note("are SB-routable to Arduino pins (not used as EXTI by firmware) -> NC here.", X1+95, 329, size=1.1)
note("Default jumpers:", X1-20, 340, size=1.4)
note("J4/J5 = 5-6  (I2C bus routing: STM I2C -> sensors)", X1-20, 344, size=1.2)
note("J2 = open    (USER_INT selector off -> sensor INTs unrouted -> NC)", X1-20, 347, size=1.2)
note("JP1/JP2 = open (Vio / BT_Irq selectors)   JP5 = 1-2 (DIL24 = 3V3_IO)", X1-20, 350, size=1.2)

# ======================= SECTION 3 : VL53L9CX (top) =======================
X2=640
note("X-NUCLEO-53L9A1  (top board)", X2-20, 22, size=3.0)
note("VL53L9CX ToF 3D LiDAR. I3C on PB8/PB9, level-shifted to sensor IOVDD.", X2-20, 28, size=1.6)
box(X2-25, 16, X2+150, 312)
emit_connector("Conn_1x10","CN205", CN5, X2, 58,  ds=DS["board_vl53"], realname="= CN5")
emit_connector("Conn_1x08","CN206", CN6, X2, 112, ds=DS["board_vl53"], realname="= CN6")
emit_connector("Conn_1x06","CN208", CN8, X2, 162, ds=DS["board_vl53"], realname="= CN8")
emit_connector("Conn_1x08","CN209", CN9, X2, 206, ds=DS["board_vl53"], realname="= CN9")

u5=place("PI4ULS3V204","U205", X2+90, 74, refy_off=-14, datasheet=DS["PI4ULS3V204"], realname="= U5")
glabel("+3V3",*u5["1"],to_right=False); glabel("PB9",*u5["2"],to_right=False); glabel("PB8",*u5["3"],to_right=False)
noconn(*u5["4"]); noconn(*u5["5"]); glabel("+3V3",*u5["8"],to_right=False); glabel("GND",*u5["7"],to_right=False)
llabel("VL53_IOVDD",*u5["14"],to_right=True); llabel("VL53_S_SDA",*u5["13"],to_right=True); llabel("VL53_S_SCL",*u5["12"],to_right=True)
noconn(*u5["11"]); noconn(*u5["10"])
note("U5 PI4ULS3V204: I3C SCL/SDA level shift, 3V3 <-> IOVDD.", X2+52, 94, size=1.2)

u6=place("PI4ULS3V204","U206", X2+90, 154, refy_off=-14, datasheet=DS["PI4ULS3V204"], realname="= U6")
glabel("+3V3",*u6["1"],to_right=False); glabel("PB1",*u6["2"],to_right=False); glabel("PB5",*u6["3"],to_right=False)
glabel("PB6",*u6["4"],to_right=False); glabel("PB7",*u6["5"],to_right=False)
glabel("+3V3",*u6["8"],to_right=False); glabel("GND",*u6["7"],to_right=False)
llabel("VL53_IOVDD",*u6["14"],to_right=True); llabel("VL53_S_SYNC",*u6["13"],to_right=True); llabel("VL53_CLK_EXT",*u6["12"],to_right=True)
llabel("VL53_S_XSHUT",*u6["11"],to_right=True); llabel("VL53_S_INTR",*u6["10"],to_right=True)
note("U6 PI4ULS3V204: SYNC/CLK/XSHUT/INTR level shift.", X2+52, 174, size=1.2)

# on-board 12MHz oscillator + SW1 clock-select switch (default SW1 = INT -> Y1)
osc=place("OSC_12MHz","Y201", X2+30, 236, refy_off=-10, datasheet=DS["OSC"], realname="= Y1")
glabel("GND",*osc["1"],to_right=False); glabel("GND",*osc["2"],to_right=False)
llabel("VL53_CLK_INT",*osc["3"],to_right=True); llabel("VL53_IOVDD",*osc["4"],to_right=True)
sw=place("SW_SPDT","SW1", X2+90, 236, refy_off=-9, realname="SW1 = INT")
llabel("VL53_CLK_INT",*sw["1"],to_right=False)   # INT throw <- Y1
llabel("VL53_CLK_EXT",*sw["3"],to_right=False)   # EXT throw <- host CLK (PB5, idle)
llabel("VL53_S_CLK",  *sw["2"],to_right=True)    # COM -> sensor AP_CLK
note("SW1=INT: ToF CLK from on-board Y1 12MHz. Host PB5/CLK_IN path idle (EXT).", X2+15, 258, size=1.2)

v=place("VL53L9CX","U207", X2+110, 250, refy_off=-16, datasheet=DS["VL53L9CX"], realname="= U7")
llabel("VL53_S_SCL",*v["A11"],to_right=False); llabel("VL53_S_SDA",*v["A12"],to_right=False)
llabel("VL53_S_INTR",*v["A10"],to_right=False); llabel("VL53_S_XSHUT",*v["B12"],to_right=False)
llabel("VL53_S_SYNC",*v["D12"],to_right=False); llabel("VL53_S_CLK",*v["E11"],to_right=False)
llabel("VL53_IOVDD",*v["E8"],to_right=True); noconn(*v["E6"]); noconn(*v["C12"])
glabel("+3V3",*v["B1"],to_right=True); glabel("GND",*v["A1"],to_right=True)
note("U7 VL53L9CX ToF (0x52). On-board LDOs (from CN6 +5V) derive", X2+70, 274, size=1.2)
note("IOVDD/AVDD(2V8)/DVDD(1V2) - AVDD/DVDD abstracted (NC).", X2+70, 277, size=1.2)
note("Default jumpers:", X2-20, 288, size=1.4)
note("SW1 = INT   (ToF CLK from on-board Y1 12MHz; see switch)", X2-20, 292, size=1.2)
note("J1 = 3V3    (Nucleo_IOVDD host-side shifter reference = 3.3V)", X2-20, 295, size=1.2)
note("J2..J5 = linked  (VBAT_LDD/VBAT_RX/AVDD/DVDD rails powered)", X2-20, 298, size=1.2)

# ----------------------------------------------------------------------------
# write files
# ----------------------------------------------------------------------------
lib=SymbolLib(); lib.version="20211014"; lib.generator="kicad_symbol_editor"
for s in libsyms.values(): lib.symbols.append(s)
lib.to_file(os.path.join(OUT, f"{NICK}.kicad_sym"))
sch.to_file(os.path.join(OUT, f"{PROJ}.kicad_sch"))

import json
kicad_pro={"board":{"design_settings":{"defaults":{},"rules":{}},"layer_presets":[],"viewports":[]},
  "boards":[],"cvpcb":{"equivalence_files":[]},
  "libraries":{"pinned_footprint_libs":[],"pinned_symbol_libs":[]},
  "meta":{"filename":f"{PROJ}.kicad_pro","version":1},
  "net_settings":{"classes":[{"name":"Default","clearance":0.2}]},
  "pcbnew":{"last_paths":{},"page_layout_descr_file":""},
  "schematic":{"annotate_start_num":0,"drawing":{"default_line_thickness":6.0,"default_text_size":50.0},
     "legacy_lib_dir":"","legacy_lib_list":[],"meta":{"version":1},"net_format_name":"",
     "spice_external_command":"spice \"%I\"","subpart_id_separator":0,"subpart_first_id":65},
  "sheets":[[str(ROOT_UUID),"Root"]],"text_variables":{}}
with open(os.path.join(OUT,f"{PROJ}.kicad_pro"),"w") as f: json.dump(kicad_pro,f,indent=2)
with open(os.path.join(OUT,"sym-lib-table"),"w") as f:
    f.write('(sym_lib_table\n  (version 7)\n'
            f'  (lib (name "{NICK}")(type "KiCad")(uri "${{KIPRJMOD}}/{NICK}.kicad_sym")(options "")(descr "Consolidated NUCLEO stack symbols"))\n)\n')
with open(os.path.join(OUT,"fp-lib-table"),"w") as f: f.write('(fp_lib_table\n  (version 7)\n)\n')
with open(os.path.join(DSDIR,"MANIFEST.md"),"w") as f: f.write("\n".join(manifest)+"\n")

print("symbols placed:", len(sch.schematicSymbols))
print("global:",len(sch.globalLabels)," local:",len(sch.labels)," nc:",len(sch.noConnects)," texts:",len(sch.texts))
print("datasheets copied to:", DSDIR)
