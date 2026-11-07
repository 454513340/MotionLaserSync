#!/usr/bin/env python3
"""V27.6.55 / BASIC V32.13 structural contract ONLY; no hardware or BASIC compilation."""
from pathlib import Path
r=Path(__file__).resolve().parents[1]
read=lambda path:(r/path).read_text(encoding="utf-8")
fw=read("controller/ECI3808_5Axis_Controller.bas")
assert fw==read("controller/ECI3808_V32_13_PWM_HANDSHAKE_FIVE_AXIS_HOME.bas")
assert fw==read("ECI3808_V32_13_PWM_HANDSHAKE_FIVE_AXIS_HOME.bas")
assert "ECI3808 V32.13" in fw and "CONST RUNTIME_MAGIC=321055" in fw
assert "TABLE(354)=RUNTIME_MAGIC" in fw and "TABLE(357)=0" in fw
assert all(f"TABLE({n})=0" in fw for n in (358,359,360,361))
assert "ENCODER_RATIO(2,1) AXIS(1)" in fw
assert "ENCODER_RATIO(2,1) AXIS(2)" in fw
assert "ENCODER_RATIO(2,-1) AXIS(3)" in fw
assert "FS_LIMIT=114,96,56" in fw and "RS_LIMIT=-150,-72,-35" in fw
idle=fw[fw.index(" IF en=0 THEN"):fw.index(" ELSEIF TABLE(340)<>TABLE(341) THEN")]
assert idle.index("StopPwmSync()") < idle.index("  gm=0\n") < idle.index("ApplyRuntimeConfig()")
assert "stopdelay=1000/gf+2" in fw and "1000/TABLE(356)" not in fw
assert fw.index("PWM_FREQ(0)=reqf") < fw.index("TABLE(360)=ga")
assert "ELSEIF TABLE(355)<>gf OR TABLE(359)<>ga OR TABLE(361)<>0 THEN" in fw
home_guard="ELSEIF TABLE(357)<>31 THEN"
assert home_guard in fw and fw.index(home_guard)<fw.index(" ELSEIF en=3 THEN")
assert "TABLE(333)=12\n  Bad()" in fw
header=read("include/runtime/Acquisition5D.h")
assert "runtimeCapabilityMagic=321055" in header
assert "integer(v[3],0,static_cast<int>(allAxesMask))" in header
controller=read("src/core/ZmcController.cpp")
assert "GetTable(five-axis home mask readback)" in controller
assert "修正过期Home位未获得控制器写后回读确认" in controller
assert "XYZ UNITS修改前无法可靠撤销控制器Home位" in controller
assert controller.count("if (!areAllAxesHomed())")>=2
assert "五轴Home未全部确认，拒绝ColdPrime触发输出" in controller
assert "等待真实停机状态时TABLE协议数据非整数" in controller
assert "pwm.frequency.requestV3212" not in controller
assert "V32.13" in read("src/app/MainWindow.cpp")
assert '"V27.6.55"' in read("src/widgets/HardwareSyncCalibrationWidget.cpp")
assert "runtimeCapabilityMagic==321055" in read("tests/v27655_runtime_pwm_tests.cpp")
print("PASS V27.6.55 / V32.13 static protocol, Home interlock, version and delivery contract")
