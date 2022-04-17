{

  Project: EE-20 Assignment
  Platform: Parallax Project USB Board
  Revision: 1.2
  Author: Teo Xin Yi
  Date: 1st Apr 2022
  Log:
        Date: Desc
        24/2/2022: Connect & Control Lite Kit using mecanum wheels
        5/3/2022:  Include sensors readings and direction verification
        16/3/2022: Integration
        1/4/2022:  Port to Production Kit

}

CON
        _clkmode                = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
        _xinfreq                = 5_000_000
        _ConClkFreq             = ((_clkmode - xtal1) >> 6) + _xinfreq
        _Ms_001                 = _ConClkFreq / 1_000


        ' Movement Hex Key
        commForward   = $01
        commReverse   = $02
        { v1.1: remove rotation movement
        commLeft      = $03
        commRight     = $04}
        commDiagNE    = $05
        commDiagSW    = $06
        commDiagNW    = $07
        commDiagSE    = $08
        commSideLeft  = $09
        commSideRight = $10
        commStopAll   = $AA

VAR    'Global Variables
  long  Mainmeca, mecspeed, dirVal, speed, tofMainMem, ultraMainMem

OBJ
  Meca          : "MecanumControl.spin"      '<-- Object / Blackbox
  Sensor        : "SensorMUXControl.spin"
  Comm2         : "Comm2Control.spin"
  Term          : "FullDuplexSerial.spin"   'UART communication for debugging

PUB Main | i

  Meca.Start(_Ms_001, @Mainmeca, @mecspeed)
  Comm2.Start(_Ms_001, @dirVal, @speed)
  Sensor.Start(@tofMainMem, @ultraMainMem)
  Term.Start(31,30,0,230400)

  repeat
    Term.Str(String(13, "DirValue: "))
    Term.Dec(dirVal)
    Mainmeca := dirVal
    mecspeed := speed
    Pause(1000)

PRI Pause(ms) | t    'Pause Function

  t := cnt - 1088
  repeat(ms #> 0)
    waitcnt(t += _Ms_001)
  return