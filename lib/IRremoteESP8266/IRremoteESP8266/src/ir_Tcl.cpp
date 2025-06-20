// Copyright 2019, 2021, 2022 David Conran

/// @file
/// @brief Support for TCL protocols.

#include "ir_Tcl.h"
#include <algorithm>
#include <cstring>
#ifndef ARDUINO
#include <string>
#endif
#include "IRremoteESP8266.h"
#include "IRtext.h"
#include "IRutils.h"

// Constants
const uint8_t kTcl112AcTimerResolution = 20;  // Minutes
const uint16_t kTcl112AcTimerMax = 720;  // Minutes (12 hrs)

const uint16_t kTcl96AcHdrMark = 1056;  // uSeconds.
const uint16_t kTcl96AcHdrSpace = 550;  // uSeconds.
const uint16_t kTcl96AcBitMark = 600;   // uSeconds.
const uint32_t kTcl96AcGap = kDefaultMessageGap;  // Just a guess.
const uint8_t  kTcl96AcSpaceCount = 4;
const uint16_t kTcl96AcBitSpaces[kTcl96AcSpaceCount] = {360,    // 0b00
                                                        838,    // 0b01
                                                        2182,   // 0b10
                                                        1444};  // 0b11

using irutils::addBoolToString;
using irutils::addFanToString;
using irutils::addIntToString;
using irutils::addLabeledString;
using irutils::addModeToString;
using irutils::addModelToString;
using irutils::addSwingVToString;
using irutils::addTempFloatToString;
using irutils::minsToString;

#if SEND_TCL112AC
/// Send a TCL 112-bit A/C message.
/// Status: Beta / Probably working.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendTcl112Ac(const unsigned char data[], const uint16_t nbytes,
                          const uint16_t repeat) {
  sendGeneric(kTcl112AcHdrMark, kTcl112AcHdrSpace,
              kTcl112AcBitMark, kTcl112AcOneSpace,
              kTcl112AcBitMark, kTcl112AcZeroSpace,
              kTcl112AcBitMark, kTcl112AcGap,
              data, nbytes, 38000, false, repeat, 50);
}
#endif  // SEND_TCL112AC

/// Class constructor
/// @param[in] pin GPIO to be used when sending.
/// @param[in] inverted Is the output signal to be inverted?
/// @param[in] use_modulation Is frequency modulation to be used?
IRTcl112Ac::IRTcl112Ac(const uint16_t pin, const bool inverted,
                       const bool use_modulation)
    : _irsend(pin, inverted, use_modulation) { stateReset(); }

/// Set up hardware to be able to send a message.
void IRTcl112Ac::begin(void) { _irsend.begin(); }

#if SEND_TCL112AC
/// Send the current internal state as an IR message.
/// @param[in] repeat Nr. of times the message will be repeated.
void IRTcl112Ac::send(const uint16_t repeat) {
  uint8_t save[kTcl112AcStateLength];
  // Do we need to send the special "quiet" message?
  if (_quiet != _quiet_prev) {
    // Backup the current state.
    std::memcpy(save, _.raw, kTcl112AcStateLength);
    const uint8_t quiet_off[kTcl112AcStateLength] = {
        0x23, 0xCB, 0x26, 0x02, 0x00, 0x40, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65};
    // Use a known good quiet/mute off/type 2 state for the time being.
    // Ref: https://github.com/crankyoldgit/IRremoteESP8266/issues/1528#issuecomment-876989044
    setRaw(quiet_off);
    setQuiet(_quiet);
    // Send it.
    _irsend.sendTcl112Ac(getRaw(), kTcl112AcStateLength, repeat);
    // Now it's been sent, update the quiet previous state.
    _quiet_prev = _quiet;
    // Restore the old state.
    setRaw(save);
    // Make sure it looks like a normal TCL mesg if needed.
  }
  // Send the normal (type 1) state.
  _irsend.sendTcl112Ac(getRaw(), kTcl112AcStateLength, repeat);
}
#endif  // SEND_TCL112AC

/// Calculate the checksum for a given state.
/// @param[in] state The array to calc the checksum of.
/// @param[in] length The length/size of the array.
/// @return The calculated checksum value.
uint8_t IRTcl112Ac::calcChecksum(uint8_t state[], const uint16_t length) {
  if (length) {
    if (length > 4 && state[3] == 0x02) {  // Special nessage?
      return sumBytes(state, length - 1, 0xF);  // Checksum needs an offset.
    } else {
      return sumBytes(state, length - 1);
    }
  } else {
    return 0;
  }
}

/// Calculate & set the checksum for the current internal state of the remote.
/// @param[in] length The length/size of the internal array to checksum.
void IRTcl112Ac::checksum(const uint16_t length) {
  // Stored the checksum value in the last byte.
  if (length > 1)
    _.Sum = calcChecksum(_.raw, length);
}

/// Verify the checksum is valid for a given state.
/// @param[in] state The array to verify the checksum of.
/// @param[in] length The length/size of the array.
/// @return true, if the state has a valid checksum. Otherwise, false.
bool IRTcl112Ac::validChecksum(uint8_t state[], const uint16_t length) {
  return (length > 1 && state[length - 1] == calcChecksum(state, length));
}


/// Reset the internal state of the emulation. (On, Cool, 24C)
void IRTcl112Ac::stateReset(void) {
  // A known good state. (On, Cool, 24C)
  static const uint8_t reset[kTcl112AcStateLength] = {
      0x23, 0xCB, 0x26, 0x01, 0x00, 0x24, 0x03, 0x07, 0x00, 0x00, 0x00, 0x00,
      0x70, 0xB3};
  std::memcpy(_.raw, reset, kTcl112AcStateLength);
  _quiet = false;
  _quiet_prev = false;
  _quiet_explictly_set = false;
}


/// Get a PTR to the internal state/code for this protocol.
/// @return PTR to a code for this protocol based on the current internal state.
uint8_t* IRTcl112Ac::getRaw(void) {
  checksum();
  return _.raw;
}

/// Set the internal state from a valid code for this protocol.
/// @param[in] new_code A valid code for this protocol.
/// @param[in] length The length/size of the new_code array.
void IRTcl112Ac::setRaw(const uint8_t new_code[], const uint16_t length) {
  std::memcpy(_.raw, new_code, std::min(length, kTcl112AcStateLength));
}

/// Set the requested power state of the A/C to on.
void IRTcl112Ac::on(void) { setPower(true); }

/// Set the requested power state of the A/C to off.
void IRTcl112Ac::off(void) { setPower(false); }

/// Change the power setting.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRTcl112Ac::setPower(const bool on) { _.Power = on; }

/// Get the value of the current power setting.
/// @return true, the setting is on. false, the setting is off.
bool IRTcl112Ac::getPower(void) const { return _.Power; }

/// Get the operating mode setting of the A/C.
/// @return The current operating mode setting.
uint8_t IRTcl112Ac::getMode(void) const { return _.Mode; }

/// Set the operating mode of the A/C.
/// @param[in] mode The desired operating mode.
/// @note Fan/Ventilation mode sets the fan speed to high.
///   Unknown values default to Auto.
void IRTcl112Ac::setMode(const uint8_t mode) {
  // If we get an unexpected mode, default to AUTO.
  switch (mode) {
    case kTcl112AcFan:
      setFan(kTcl112AcFanHigh);
      // FALLTHRU
    case kTcl112AcAuto:
    case kTcl112AcCool:
    case kTcl112AcHeat:
    case kTcl112AcDry:
      _.Mode = mode;
      break;
    default:
      _.Mode = kTcl112AcAuto;
  }
}

/// Set the temperature.
/// @param[in] celsius The temperature in degrees celsius.
/// @note The temperature resolution is now 1°.
void IRTcl112Ac::setTemp(const float celsius) {
  // Закръгляме до най-близкото цяло и го ограничаваме в [kTcl112AcTempMin..kTcl112AcTempMax].
  int safeC = static_cast<int>(roundf(celsius));
  safeC = std::max(safeC, static_cast<int>(kTcl112AcTempMin));
  safeC = std::min(safeC, static_cast<int>(kTcl112AcTempMax));
  // Записваме като отместване от max (цяла степен).
  _.Temp = static_cast<uint8_t>(kTcl112AcTempMax - safeC);
}

/// Get the current temperature setting.
/// @return The setting for temp. in degrees celsius (цяла степен).
float IRTcl112Ac::getTemp(void) const {
  return static_cast<float>(kTcl112AcTempMax - _.Temp);
}

/// Set the speed of the fan.
/// @param[in] speed The desired setting.
/// @note Unknown speeds will default to Auto.
void IRTcl112Ac::setFan(const uint8_t speed) {
  switch (speed) {
    case kTcl112AcFanAuto:
    case kTcl112AcFanLow:
    case kTcl112AcFanMed:
    case kTcl112AcFanHigh:
      _.Fan = speed;
      break;
    default:
      _.Fan = kTcl112AcFanAuto;
  }
}

void IRTcl112Ac::setDryClean(const bool on) {
  _.DryClean = on;
}

bool IRTcl112Ac::getDryClean(void) const {
  return _.DryClean;
}

/// Включи нощен режим
void IRTcl112Ac::nightOn(void) { setFanNight(true); }

/// Изключи нощен режим
void IRTcl112Ac::nightOff(void) { setFanNight(false); }

/// Включи/изключи нощен режим
void IRTcl112Ac::setFanNight(const bool on) {
  if (on) {
    _previousFan = _.Fan;                 // запомни текущия Fan
    _.Fan = kTcl112AcFanNight;
  } else {
    _.Fan = _previousFan;                 // върни предишната стойност
  }
}

/// Връща текущото състояние на нощния режим
bool IRTcl112Ac::getFanNight(void) const {
  return _.Fan == kTcl112AcFanNight;
}

/// Включи турбо режим
void IRTcl112Ac::turboOn(void) { setFanTurbo(true); }

/// Изключи турбо режим
void IRTcl112Ac::turboOff(void) { setFanTurbo(false); }

/// Включи/изключи турбо режим
void IRTcl112Ac::setFanTurbo(const bool on) {
  if (on) {
    _previousFan = _.Fan;                 // запомни текущия Fan
    _.Fan = kTcl112AcFanTurbo;
  } else {
    _.Fan = _previousFan;                 // върни предишната стойност
  }
}

/// Връща текущото състояние на турбо режима
bool IRTcl112Ac::getFanTurbo(void) const {
  return _.Fan == kTcl112AcFanTurbo;
}

/// Set display light flag (бит 3 от байт 5)
void IRTcl112Ac::setDisplayLight(const bool on) {
  _.DisplayLight = on;
}

/// Get display light flag
bool IRTcl112Ac::getDisplayLight(void) const {
  return _.DisplayLight;
}

/// Set Light On flag (бит 5 от байт 6 — споделен с Turbo)
void IRTcl112Ac::setLightOn(const bool on) {
  _.LightOn = on;
}

/// Get Light On flag
bool IRTcl112Ac::getLightOn(void) const {
  return _.LightOn;
}

/// Задай флаг за изключване на дисплея (бит 7 в байт 6)
void IRTcl112Ac::setLightOff(const bool on) {
  _.LightOff = on;
}

/// Върни текущото състояние на флага за изключване на дисплея
bool IRTcl112Ac::getLightOff(void) const {
  return _.LightOff;
}

/// Set full display flag byte (байт 9)
void IRTcl112Ac::setDisplayFlags(const uint8_t val) {
  _.DisplayFlags = val;
}

/// Get full display flag byte
uint8_t IRTcl112Ac::getDisplayFlags(void) const {
  return _.DisplayFlags;
}


/// Get the current fan speed setting.
/// @return The current fan speed/mode.
uint8_t IRTcl112Ac::getFan(void) const { return _.Fan; }

/// Set the economy setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRTcl112Ac::setEcono(const bool on) { _.Econo = on; }

/// Get the economy setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRTcl112Ac::getEcono(void) const { return  _.Econo; }

/// Set the Health (Filter) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRTcl112Ac::setHealth(const bool on) { _.Health = on; }

/// Get the Health (Filter) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRTcl112Ac::getHealth(void) const { return _.Health; }

/// Set the Light (LED/Display) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRTcl112Ac::setLight(const bool on) { _.Light = !on; }  // Cleared when on.

/// Get the Light (LED/Display) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRTcl112Ac::getLight(void) const { return !_.Light; }

/// Set the horizontal swing setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
//void IRTcl112Ac::setSwingHorizontal(const bool on) { _.SwingH = on; }  //стара версия
//нова версия
/// Задава horizontal swing позицията (константа 1…7).
void IRTcl112Ac::setSwingHorizontal(const uint8_t position) {
  // Вземаме само ниските 3 бита от константата:
  const uint8_t v3 = position & 0x07;
  // Вграждаме в старшия nibble на raw[12]:
  _.raw[12] = static_cast<uint8_t>((_.raw[12] & 0x0F) | (v3 << 4));
}

/// Set the vertical swing setting of the A/C.
/// @param[in] setting The value of the desired setting.
/// Връща horizontal swing позиция (1…7).
uint8_t IRTcl112Ac::getSwingHorizontal(void) const {
  return static_cast<uint8_t>((_.raw[12] >> 4) & 0x07);
}

/// Set the vertical swing setting of the A/C.
/// @param[in] setting The value of the desired setting.
void IRTcl112Ac::setSwingVertical(const uint8_t setting) {	
  switch (setting) {
    case kTcl112AcSwingVOff:
    case kTcl112AcSwingVHighest:
    case kTcl112AcSwingVHigh:
    case kTcl112AcSwingVMiddle:
    case kTcl112AcSwingVLow:
    case kTcl112AcSwingVLowest:
    case kTcl112AcSwingVOn:
     _.SwingV = setting;
  }
}

/// Get the vertical swing setting of the A/C.
/// @return The current setting.
uint8_t IRTcl112Ac::getSwingVertical(void) const { return _.SwingV; }

/// Set the Turbo setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRTcl112Ac::setTurbo(const bool on) {
  _.Turbo = on;
  if (on) {
    _.Fan = kTcl112AcFanHigh;
    _.SwingV = kTcl112AcSwingVOn;
  }
}

/// Get the Turbo setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRTcl112Ac::getTurbo(void) const { return _.Turbo; }

/// Set the Quiet setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRTcl112Ac::setQuiet(const bool on) {
  _quiet_explictly_set = true;
  _quiet = on;
  if (_.MsgType == kTcl112AcSpecial) _.Quiet = on;
}

/// Get the Quiet setting of the A/C.
/// @param[in] def The default value to use if we are not sure.
/// @return true, the setting is on. false, the setting is off.
bool IRTcl112Ac::getQuiet(const bool def) const {
  if (_.MsgType == kTcl112AcSpecial)
    return _.Quiet;
  else
    return _quiet_explictly_set ? _quiet : def;
}

/// Get how long the On Timer is set for, in minutes.
/// @return The time in nr of minutes.
uint16_t IRTcl112Ac::getOnTimer(void) const {
  return _.OnTimer * kTcl112AcTimerResolution;
}

/// Set or cancel the On Timer function.
/// @param[in] mins Nr. of minutes the timer is to be set to.
/// @note Rounds down to 20 min increments. (max: 720 mins (12h), 0 is Off)
void IRTcl112Ac::setOnTimer(const uint16_t mins) {
  _.OnTimer = std::min(mins, kTcl112AcTimerMax) / kTcl112AcTimerResolution;
  _.OnTimerEnabled = _.OnTimer > 0;
  _.TimerIndicator = _.OnTimerEnabled || _.OffTimerEnabled;
}

/// Get how long the Off Timer is set for, in minutes.
/// @return The time in nr of minutes.
uint16_t IRTcl112Ac::getOffTimer(void) const {
  return _.OffTimer * kTcl112AcTimerResolution;
}

/// Set or cancel the Off Timer function.
/// @param[in] mins Nr. of minutes the timer is to be set to.
/// @note Rounds down to 20 min increments. (max: 720 mins (12h), 0 is Off)
void IRTcl112Ac::setOffTimer(const uint16_t mins) {
  _.OffTimer = std::min(mins, kTcl112AcTimerMax) / kTcl112AcTimerResolution;
  _.OffTimerEnabled = _.OffTimer > 0;
  _.TimerIndicator = _.OnTimerEnabled || _.OffTimerEnabled;
}

/// Convert a stdAc::opmode_t enum into its native mode.
/// @param[in] mode The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRTcl112Ac::convertMode(const stdAc::opmode_t mode) {
  switch (mode) {
    case stdAc::opmode_t::kCool: return kTcl112AcCool;
    case stdAc::opmode_t::kHeat: return kTcl112AcHeat;
    case stdAc::opmode_t::kDry:  return kTcl112AcDry;
    case stdAc::opmode_t::kFan:  return kTcl112AcFan;
    default:                     return kTcl112AcAuto;
  }
}

/// Convert a stdAc::fanspeed_t enum into it's native speed.
/// @param[in] speed The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRTcl112Ac::convertFan(const stdAc::fanspeed_t speed) {
  switch (speed) {
    case stdAc::fanspeed_t::kMin:    return kTcl112AcFanMin;
    case stdAc::fanspeed_t::kLow:    return kTcl112AcFanLow;
    case stdAc::fanspeed_t::kMedium: return kTcl112AcFanMed;
    case stdAc::fanspeed_t::kHigh:
    case stdAc::fanspeed_t::kMax:    return kTcl112AcFanHigh;
    default:                         return kTcl112AcFanAuto;
  }
}

/// Convert a native mode into its stdAc equivalent.
/// @param[in] mode The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::opmode_t IRTcl112Ac::toCommonMode(const uint8_t mode) {
  switch (mode) {
    case kTcl112AcCool: return stdAc::opmode_t::kCool;
    case kTcl112AcHeat: return stdAc::opmode_t::kHeat;
    case kTcl112AcDry:  return stdAc::opmode_t::kDry;
    case kTcl112AcFan:  return stdAc::opmode_t::kFan;
    default:            return stdAc::opmode_t::kAuto;
  }
}

/// Convert a stdAc::swingv_t enum into it's native setting.
/// @param[in] position The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRTcl112Ac::convertSwingV(const stdAc::swingv_t position) {
  switch (position) {
    case stdAc::swingv_t::kOff:     return kTcl112AcSwingVOff;
    case stdAc::swingv_t::kHighest: return kTcl112AcSwingVHighest;
    case stdAc::swingv_t::kHigh:    return kTcl112AcSwingVHigh;
    case stdAc::swingv_t::kMiddle:  return kTcl112AcSwingVMiddle;
    case stdAc::swingv_t::kLow:     return kTcl112AcSwingVLow;
    case stdAc::swingv_t::kLowest:  return kTcl112AcSwingVLowest;
    default:                        return kTcl112AcSwingVOn;
  }
}

/// Convert a native fan speed into its stdAc equivalent.
/// @param[in] spd The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::fanspeed_t IRTcl112Ac::toCommonFanSpeed(const uint8_t spd) {
  switch (spd) {
    case kTcl112AcFanHigh: return stdAc::fanspeed_t::kMax;
    case kTcl112AcFanMed:  return stdAc::fanspeed_t::kMedium;
    case kTcl112AcFanLow:  return stdAc::fanspeed_t::kLow;
    case kTcl112AcFanMin:  return stdAc::fanspeed_t::kMin;
    default:               return stdAc::fanspeed_t::kAuto;
  }
}

/// Convert a native vertical swing postion to it's common equivalent.
/// @param[in] setting A native position to convert.
/// @return The common vertical swing position.
stdAc::swingv_t IRTcl112Ac::toCommonSwingV(const uint8_t setting) {
  switch (setting) {
    case kTcl112AcSwingVOff:       return stdAc::swingv_t::kOff;
    default:                       return stdAc::swingv_t::kAuto;
  }
}
/// Convert the current internal state into its stdAc::state_t equivalent.
/// @param[in] prev Ptr to the previous state if required.
/// @return The stdAc equivalent of the native settings.
stdAc::state_t IRTcl112Ac::toCommon(const stdAc::state_t *prev) const {
  stdAc::state_t result{};
  // Start with the previous state if given it.
  if (prev != NULL) result = *prev;
  result.protocol = decode_type_t::TCL112AC;
  result.model = getModel();
  result.quiet = getQuiet(result.quiet);
  // The rest only get updated if it is a "normal" message.
  if (_.MsgType == kTcl112AcNormal) {
    result.power = _.Power;
    result.mode = toCommonMode(_.Mode);
    result.celsius = true;
    result.degrees = getTemp();
    result.fanspeed = toCommonFanSpeed(_.Fan);
    result.swingv = toCommonSwingV(_.SwingV);
    result.swingh = _.SwingH ? stdAc::swingh_t::kAuto : stdAc::swingh_t::kOff;
    result.turbo = _.Turbo;
    result.filter = _.Health;
    result.econo = _.Econo;
    result.light = getLight();
  }
  // Not supported.
  result.clean = false;
  result.beep = false;
  result.sleep = -1;
  result.clock = -1;
  return result;
}

/// Convert the current internal state into a human readable string.
/// @return A human readable string.
String IRTcl112Ac::toString(void) const {
  String result = "";
  result.reserve(220);  // Reserve some heap for the string to reduce fragging.
  tcl_ac_remote_model_t model = getModel();
  result += addModelToString(decode_type_t::TCL112AC, model, false);
  result += addIntToString(_.MsgType, D_STR_TYPE);
  switch (_.MsgType) {
    case kTcl112AcNormal:
      result += addBoolToString(_.Power, kPowerStr);
      result += addModeToString(_.Mode, kTcl112AcAuto, kTcl112AcCool,
                                kTcl112AcHeat, kTcl112AcDry, kTcl112AcFan);
      result += addTempFloatToString(getTemp());
      result += addFanToString(_.Fan, kTcl112AcFanHigh, kTcl112AcFanLow,
                               kTcl112AcFanAuto, kTcl112AcFanMin,
                               kTcl112AcFanMed);
      result += addSwingVToString(_.SwingV, kTcl112AcSwingVOff,
                                            kTcl112AcSwingVHighest,
                                            kTcl112AcSwingVHigh,
                                            0xFF,  // unused
                                            kTcl112AcSwingVMiddle,
                                            0xFF,  // unused
                                            kTcl112AcSwingVLow,
                                            kTcl112AcSwingVLowest,
                                            kTcl112AcSwingVOff,
                                            kTcl112AcSwingVOn,  // Swing
                                            0xFF, 0xFF);  // Both Unused
      if (model != tcl_ac_remote_model_t::GZ055BE1) {
        result += addBoolToString(_.SwingH, kSwingHStr);
        result += addBoolToString(_.Econo, kEconoStr);
        result += addBoolToString(_.Health, kHealthStr);
        result += addBoolToString(_.Turbo, kTurboStr);
        result += addBoolToString(getLight(), kLightStr);
      }
      result += addLabeledString(
          _.OnTimerEnabled ? minsToString(getOnTimer()) : kOffStr,
          kOnTimerStr);
      result += addLabeledString(
          _.OffTimerEnabled ? minsToString(getOffTimer()) : kOffStr,
          kOffTimerStr);
      break;
    case kTcl112AcSpecial:
      result += addBoolToString(_.Quiet, kQuietStr);
      break;
  }
  return result;
}

#if DECODE_TCL112AC
/// @file
/// @note There is no `decodedecodeTcl112Ac()`.
///   It's the same as `decodeMitsubishi112()`. A shared routine is used.
///   You can find it in: ir_Mitsubishi.cpp
#endif  // DECODE_TCL112AC

#if SEND_TCL96AC
/// Send a TCL 96-bit A/C message.
/// Status: BETA / Untested on a real device working.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendTcl96Ac(const unsigned char data[], const uint16_t nbytes,
                         const uint16_t repeat) {
  enableIROut(38);
  for (uint16_t r = 0; r <= repeat; r++) {
    // Header
    mark(kTcl96AcHdrMark);
    space(kTcl96AcHdrSpace);
    // Data
    for (uint16_t pos = 0; pos < nbytes; pos++) {
      uint8_t databyte = data[pos];
      for (uint8_t bits = 0; bits < 8; bits += 2) {
        mark(kTcl96AcBitMark);
        space(kTcl96AcBitSpaces[GETBITS8(databyte, 8 - 2, 2)]);
        databyte <<= 2;
      }
    }
    // Footer
    mark(kTcl96AcBitMark);
    space(kTcl96AcGap);
  }
}
#endif  // SEND_TCL96AC

#if DECODE_TCL96AC
/// Decode the supplied Tcl96Ac message.
/// Status: ALPHA / Experimental.
/// @param[in,out] results Ptr to the data to decode & where to store the result
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return True if it can decode it, false if it can't.
bool IRrecv::decodeTcl96Ac(decode_results* results, uint16_t offset,
                           const uint16_t nbits, const bool strict) {
  if (results->rawlen < nbits + kHeader + kFooter - 1 + offset)
    return false;  // Message is smaller than we expected.
  if (strict && nbits != kTcl96AcBits)
    return false;  // Not strictly a TCL96AC message.
  uint8_t data = 0;
  // Header.
  if (!matchMark(results->rawbuf[offset++], kTcl96AcHdrMark)) return false;
  if (!matchSpace(results->rawbuf[offset++], kTcl96AcHdrSpace)) return false;
  // Data (2 bits at a time)
  for (uint16_t bits_so_far = 0; bits_so_far < nbits; bits_so_far += 2) {
    if (bits_so_far % 8)
      data <<= 2;  // Make space for the new data bits.
    else
      data = 0;
    if (!matchMark(results->rawbuf[offset++], kTcl96AcBitMark)) return false;
    uint8_t value = 0;
    while (value < kTcl96AcSpaceCount) {
      if (matchSpace(results->rawbuf[offset], kTcl96AcBitSpaces[value])) {
        data += value;
        break;
      }
      value++;
    }
    if (value >= kTcl96AcSpaceCount) return false;  // No matches.
    offset++;
    *(results->state + bits_so_far / 8) = data;
  }
  // Footer
  if (!matchMark(results->rawbuf[offset++], kTcl96AcBitMark)) return false;
  if (offset < results->rawlen &&
    !matchAtLeast(results->rawbuf[offset], kTcl96AcGap)) return false;
  // Success
  results->decode_type = TCL96AC;
  results->bits = nbits;
  return true;
}
#endif  // DECODE_TCL96AC
