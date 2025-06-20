// Copyright 2019, 2021, 2022 David Conran

/// @file
/// @brief Support for Osaka protocols.

#include "ir_Osaka.h"
#include <algorithm>
#include <cstring>
#ifndef ARDUINO
#include <string>
#endif
#include "IRremoteESP8266.h"
#include "IRtext.h"
#include "IRutils.h"

// Constants
const uint8_t kOsaka112AcTimerResolution = 20;  // Minutes
const uint16_t kOsaka112AcTimerMax = 720;  // Minutes (12 hrs)

const uint16_t kOsaka96AcHdrMark = 1056;  // uSeconds.
const uint16_t kOsaka96AcHdrSpace = 550;  // uSeconds.
const uint16_t kOsaka96AcBitMark = 600;   // uSeconds.
const uint32_t kOsaka96AcGap = kDefaultMessageGap;  // Just a guess.
const uint8_t  kOsaka96AcSpaceCount = 4;
const uint16_t kOsaka96AcBitSpaces[kOsaka96AcSpaceCount] = {360,    // 0b00
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

#if SEND_Osaka112AC
/// Send a Osaka 112-bit A/C message.
/// Status: Beta / Probably working.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendOsaka112Ac(const unsigned char data[], const uint16_t nbytes,
                          const uint16_t repeat) {
  sendGeneric(kOsaka112AcHdrMark, kOsaka112AcHdrSpace,
              kOsaka112AcBitMark, kOsaka112AcOneSpace,
              kOsaka112AcBitMark, kOsaka112AcZeroSpace,
              kOsaka112AcBitMark, kOsaka112AcGap,
              data, nbytes, 38000, false, repeat, 50);
}
#endif  // SEND_Osaka112AC

/// Class constructor
/// @param[in] pin GPIO to be used when sending.
/// @param[in] inverted Is the output signal to be inverted?
/// @param[in] use_modulation Is frequency modulation to be used?
IROsaka112Ac::IROsaka112Ac(const uint16_t pin, const bool inverted,
                       const bool use_modulation)
    : _irsend(pin, inverted, use_modulation) { stateReset(); }

/// Set up hardware to be able to send a message.
void IROsaka112Ac::begin(void) { _irsend.begin(); }

#if SEND_Osaka112AC
/// Send the current internal state as an IR message.
/// @param[in] repeat Nr. of times the message will be repeated.
void IROsaka112Ac::send(const uint16_t repeat) {
  uint8_t save[kOsaka112AcStateLength];
  // Do we need to send the special "quiet" message?
  if (_quiet != _quiet_prev) {
    // Backup the current state.
    std::memcpy(save, _.raw, kOsaka112AcStateLength);
    const uint8_t quiet_off[kOsaka112AcStateLength] = {
        0x23, 0xCB, 0x26, 0x02, 0x00, 0x40, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65};
    // Use a known good quiet/mute off/type 2 state for the time being.
    // Ref: https://github.com/crankyoldgit/IRremoteESP8266/issues/1528#issuecomment-876989044
    setRaw(quiet_off);
    setQuiet(_quiet);
    // Send it.
    _irsend.sendOsaka112Ac(getRaw(), kOsaka112AcStateLength, repeat);
    // Now it's been sent, update the quiet previous state.
    _quiet_prev = _quiet;
    // Restore the old state.
    setRaw(save);
    // Make sure it looks like a normal Osaka mesg if needed.
  }
  // Send the normal (type 1) state.
  _irsend.sendOsaka112Ac(getRaw(), kOsaka112AcStateLength, repeat);
}
#endif  // SEND_Osaka112AC

/// Calculate the checksum for a given state.
/// @param[in] state The array to calc the checksum of.
/// @param[in] length The length/size of the array.
/// @return The calculated checksum value.
uint8_t IROsaka112Ac::calcChecksum(uint8_t state[], const uint16_t length) {
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
void IROsaka112Ac::checksum(const uint16_t length) {
  // Stored the checksum value in the last byte.
  if (length > 1)
    _.Sum = calcChecksum(_.raw, length);
}

/// Verify the checksum is valid for a given state.
/// @param[in] state The array to verify the checksum of.
/// @param[in] length The length/size of the array.
/// @return true, if the state has a valid checksum. Otherwise, false.
bool IROsaka112Ac::validChecksum(uint8_t state[], const uint16_t length) {
  return (length > 1 && state[length - 1] == calcChecksum(state, length));
}


/// Reset the internal state of the emulation. (On, Cool, 24C)
void IROsaka112Ac::stateReset(void) {
  // A known good state. (On, Cool, 24C)
  static const uint8_t reset[kOsaka112AcStateLength] = {
      0x23, 0xCB, 0x26, 0x01, 0x00, 0x24, 0x03, 0x07, 0x00, 0x00, 0x00, 0x00,
      0x70, 0xB3};
  std::memcpy(_.raw, reset, kOsaka112AcStateLength);
  _quiet = false;
  _quiet_prev = false;
  _quiet_explictly_set = false;
}


/// Get a PTR to the internal state/code for this protocol.
/// @return PTR to a code for this protocol based on the current internal state.
uint8_t* IROsaka112Ac::getRaw(void) {
  checksum();
  return _.raw;
}

/// Set the internal state from a valid code for this protocol.
/// @param[in] new_code A valid code for this protocol.
/// @param[in] length The length/size of the new_code array.
void IROsaka112Ac::setRaw(const uint8_t new_code[], const uint16_t length) {
  std::memcpy(_.raw, new_code, std::min(length, kOsaka112AcStateLength));
}

/// Set the requested power state of the A/C to on.
void IROsaka112Ac::on(void) { setPower(true); }

/// Set the requested power state of the A/C to off.
void IROsaka112Ac::off(void) { setPower(false); }

/// Change the power setting.
/// @param[in] on true, the setting is on. false, the setting is off.
void IROsaka112Ac::setPower(const bool on) { _.Power = on; }

/// Get the value of the current power setting.
/// @return true, the setting is on. false, the setting is off.
bool IROsaka112Ac::getPower(void) const { return _.Power; }

/// Get the operating mode setting of the A/C.
/// @return The current operating mode setting.
uint8_t IROsaka112Ac::getMode(void) const { return _.Mode; }

/// Set the operating mode of the A/C.
/// @param[in] mode The desired operating mode.
/// @note Fan/Ventilation mode sets the fan speed to high.
///   Unknown values default to Auto.
void IROsaka112Ac::setMode(const uint8_t mode) {
  // If we get an unexpected mode, default to AUTO.
  switch (mode) {
    case kOsaka112AcFan:
      setFan(kOsaka112AcFanHigh);
      // FALLTHRU
    case kOsaka112AcAuto:
    case kOsaka112AcCool:
    case kOsaka112AcHeat:
    case kOsaka112AcDry:
      _.Mode = mode;
      break;
    default:
      _.Mode = kOsaka112AcAuto;
  }
}

/// Set the temperature.
/// @param[in] celsius The temperature in degrees celsius.
/// @note The temperature resolution is now 1°.
void IROsaka112Ac::setTemp(const float celsius) {
  // Закръгляме до най-близкото цяло и го ограничаваме в [kOsaka112AcTempMin..kOsaka112AcTempMax].
  int safeC = static_cast<int>(roundf(celsius));
  safeC = std::max(safeC, static_cast<int>(kOsaka112AcTempMin));
  safeC = std::min(safeC, static_cast<int>(kOsaka112AcTempMax));
  // Записваме като отместване от max (цяла степен).
  _.Temp = static_cast<uint8_t>(kOsaka112AcTempMax - safeC);
}

/// Get the current temperature setting.
/// @return The setting for temp. in degrees celsius (цяла степен).
float IROsaka112Ac::getTemp(void) const {
  return static_cast<float>(kOsaka112AcTempMax - _.Temp);
}

/// Set the speed of the fan.
/// @param[in] speed The desired setting.
/// @note Unknown speeds will default to Auto.
void IROsaka112Ac::setFan(const uint8_t speed) {
  switch (speed) {
    case kOsaka112AcFanAuto:
    case kOsaka112AcFanLow:
    case kOsaka112AcFanMed:
    case kOsaka112AcFanHigh:
      _.Fan = speed;
      break;
    default:
      _.Fan = kOsaka112AcFanAuto;
  }
}

void IROsaka112Ac::setDryClean(const bool on) {
  _.DryClean = on;
}

bool IROsaka112Ac::getDryClean(void) const {
  return _.DryClean;
}

/// Включи нощен режим
void IROsaka112Ac::nightOn(void) { setFanNight(true); }

/// Изключи нощен режим
void IROsaka112Ac::nightOff(void) { setFanNight(false); }

/// Включи/изключи нощен режим
void IROsaka112Ac::setFanNight(const bool on) {
  if (on) {
    _previousFan = _.Fan;                 // запомни текущия Fan
    _.Fan = kOsaka112AcFanNight;
  } else {
    _.Fan = _previousFan;                 // върни предишната стойност
  }
}

/// Връща текущото състояние на нощния режим
bool IROsaka112Ac::getFanNight(void) const {
  return _.Fan == kOsaka112AcFanNight;
}

/// Включи турбо режим
void IROsaka112Ac::turboOn(void) { setFanTurbo(true); }

/// Изключи турбо режим
void IROsaka112Ac::turboOff(void) { setFanTurbo(false); }

/// Включи/изключи турбо режим
void IROsaka112Ac::setFanTurbo(const bool on) {
  if (on) {
    _previousFan = _.Fan;                 // запомни текущия Fan
    _.Fan = kOsaka112AcFanTurbo;
  } else {
    _.Fan = _previousFan;                 // върни предишната стойност
  }
}

/// Връща текущото състояние на турбо режима
bool IROsaka112Ac::getFanTurbo(void) const {
  return _.Fan == kOsaka112AcFanTurbo;
}

/// Set display light flag (бит 3 от байт 5)
void IROsaka112Ac::setDisplayLight(const bool on) {
  _.DisplayLight = on;
}

/// Get display light flag
bool IROsaka112Ac::getDisplayLight(void) const {
  return _.DisplayLight;
}

/// Set Light On flag (бит 5 от байт 6 — споделен с Turbo)
void IROsaka112Ac::setLightOn(const bool on) {
  _.LightOn = on;
}

/// Get Light On flag
bool IROsaka112Ac::getLightOn(void) const {
  return _.LightOn;
}

/// Задай флаг за изключване на дисплея (бит 7 в байт 6)
void IROsaka112Ac::setLightOff(const bool on) {
  _.LightOff = on;
}

/// Върни текущото състояние на флага за изключване на дисплея
bool IROsaka112Ac::getLightOff(void) const {
  return _.LightOff;
}

/// Set full display flag byte (байт 9)
void IROsaka112Ac::setDisplayFlags(const uint8_t val) {
  _.DisplayFlags = val;
}

/// Get full display flag byte
uint8_t IROsaka112Ac::getDisplayFlags(void) const {
  return _.DisplayFlags;
}


/// Get the current fan speed setting.
/// @return The current fan speed/mode.
uint8_t IROsaka112Ac::getFan(void) const { return _.Fan; }

/// Set the economy setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IROsaka112Ac::setEcono(const bool on) { _.Econo = on; }

/// Get the economy setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IROsaka112Ac::getEcono(void) const { return  _.Econo; }

/// Set the Health (Filter) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IROsaka112Ac::setHealth(const bool on) { _.Health = on; }

/// Get the Health (Filter) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IROsaka112Ac::getHealth(void) const { return _.Health; }

/// Set the Light (LED/Display) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IROsaka112Ac::setLight(const bool on) { _.Light = !on; }  // Cleared when on.

/// Get the Light (LED/Display) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IROsaka112Ac::getLight(void) const { return !_.Light; }

/// Set the horizontal swing setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
//void IROsaka112Ac::setSwingHorizontal(const bool on) { _.SwingH = on; }  //стара версия
//нова версия
/// Задава horizontal swing позицията (константа 1…7).
void IROsaka112Ac::setSwingHorizontal(const uint8_t position) {
  // Вземаме само ниските 3 бита от константата:
  const uint8_t v3 = position & 0x07;
  // Вграждаме в старшия nibble на raw[12]:
  _.raw[12] = static_cast<uint8_t>((_.raw[12] & 0x0F) | (v3 << 4));
}

/// Set the vertical swing setting of the A/C.
/// @param[in] setting The value of the desired setting.
/// Връща horizontal swing позиция (1…7).
uint8_t IROsaka112Ac::getSwingHorizontal(void) const {
  return static_cast<uint8_t>((_.raw[12] >> 4) & 0x07);
}

/// Set the vertical swing setting of the A/C.
/// @param[in] setting The value of the desired setting.
void IROsaka112Ac::setSwingVertical(const uint8_t setting) {	
  switch (setting) {
    case kOsaka112AcSwingVOff:
    case kOsaka112AcSwingVHighest:
    case kOsaka112AcSwingVHigh:
    case kOsaka112AcSwingVMiddle:
    case kOsaka112AcSwingVLow:
    case kOsaka112AcSwingVLowest:
    case kOsaka112AcSwingVOn:
     _.SwingV = setting;
  }
}

/// Get the vertical swing setting of the A/C.
/// @return The current setting.
uint8_t IROsaka112Ac::getSwingVertical(void) const { return _.SwingV; }

/// Set the Turbo setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IROsaka112Ac::setTurbo(const bool on) {
  _.Turbo = on;
  if (on) {
    _.Fan = kOsaka112AcFanHigh;
    _.SwingV = kOsaka112AcSwingVOn;
  }
}

/// Get the Turbo setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IROsaka112Ac::getTurbo(void) const { return _.Turbo; }

/// Set the Quiet setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IROsaka112Ac::setQuiet(const bool on) {
  _quiet_explictly_set = true;
  _quiet = on;
  if (_.MsgType == kOsaka112AcSpecial) _.Quiet = on;
}

/// Get the Quiet setting of the A/C.
/// @param[in] def The default value to use if we are not sure.
/// @return true, the setting is on. false, the setting is off.
bool IROsaka112Ac::getQuiet(const bool def) const {
  if (_.MsgType == kOsaka112AcSpecial)
    return _.Quiet;
  else
    return _quiet_explictly_set ? _quiet : def;
}

Osaka_ac_remote_model_t IROsaka112Ac::getModel(void) const {
  return _.Model;  // Assuming you have _.Model in your union
}

void IROsaka112Ac::setModel(const Osaka_ac_remote_model_t model) {
  _.Model = model;
}

/// Get how long the On Timer is set for, in minutes.
/// @return The time in nr of minutes.
uint16_t IROsaka112Ac::getOnTimer(void) const {
  return _.OnTimer * kOsaka112AcTimerResolution;
}

/// Set or cancel the On Timer function.
/// @param[in] mins Nr. of minutes the timer is to be set to.
/// @note Rounds down to 20 min increments. (max: 720 mins (12h), 0 is Off)
void IROsaka112Ac::setOnTimer(const uint16_t mins) {
  _.OnTimer = std::min(mins, kOsaka112AcTimerMax) / kOsaka112AcTimerResolution;
  _.OnTimerEnabled = _.OnTimer > 0;
  _.TimerIndicator = _.OnTimerEnabled || _.OffTimerEnabled;
}

/// Get how long the Off Timer is set for, in minutes.
/// @return The time in nr of minutes.
uint16_t IROsaka112Ac::getOffTimer(void) const {
  return _.OffTimer * kOsaka112AcTimerResolution;
}

/// Set or cancel the Off Timer function.
/// @param[in] mins Nr. of minutes the timer is to be set to.
/// @note Rounds down to 20 min increments. (max: 720 mins (12h), 0 is Off)
void IROsaka112Ac::setOffTimer(const uint16_t mins) {
  _.OffTimer = std::min(mins, kOsaka112AcTimerMax) / kOsaka112AcTimerResolution;
  _.OffTimerEnabled = _.OffTimer > 0;
  _.TimerIndicator = _.OnTimerEnabled || _.OffTimerEnabled;
}

/// Convert a stdAc::opmode_t enum into its native mode.
/// @param[in] mode The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IROsaka112Ac::convertMode(const stdAc::opmode_t mode) {
  switch (mode) {
    case stdAc::opmode_t::kCool: return kOsaka112AcCool;
    case stdAc::opmode_t::kHeat: return kOsaka112AcHeat;
    case stdAc::opmode_t::kDry:  return kOsaka112AcDry;
    case stdAc::opmode_t::kFan:  return kOsaka112AcFan;
    default:                     return kOsaka112AcAuto;
  }
}

/// Convert a stdAc::fanspeed_t enum into it's native speed.
/// @param[in] speed The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IROsaka112Ac::convertFan(const stdAc::fanspeed_t speed) {
  switch (speed) {
    case stdAc::fanspeed_t::kMin:    return kOsaka112AcFanMin;
    case stdAc::fanspeed_t::kLow:    return kOsaka112AcFanLow;
    case stdAc::fanspeed_t::kMedium: return kOsaka112AcFanMed;
    case stdAc::fanspeed_t::kHigh:
    case stdAc::fanspeed_t::kMax:    return kOsaka112AcFanHigh;
    default:                         return kOsaka112AcFanAuto;
  }
}

/// Convert a native mode into its stdAc equivalent.
/// @param[in] mode The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::opmode_t IROsaka112Ac::toCommonMode(const uint8_t mode) {
  switch (mode) {
    case kOsaka112AcCool: return stdAc::opmode_t::kCool;
    case kOsaka112AcHeat: return stdAc::opmode_t::kHeat;
    case kOsaka112AcDry:  return stdAc::opmode_t::kDry;
    case kOsaka112AcFan:  return stdAc::opmode_t::kFan;
    default:            return stdAc::opmode_t::kAuto;
  }
}

/// Convert a stdAc::swingv_t enum into it's native setting.
/// @param[in] position The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IROsaka112Ac::convertSwingV(const stdAc::swingv_t position) {
  switch (position) {
    case stdAc::swingv_t::kOff:     return kOsaka112AcSwingVOff;
    case stdAc::swingv_t::kHighest: return kOsaka112AcSwingVHighest;
    case stdAc::swingv_t::kHigh:    return kOsaka112AcSwingVHigh;
    case stdAc::swingv_t::kMiddle:  return kOsaka112AcSwingVMiddle;
    case stdAc::swingv_t::kLow:     return kOsaka112AcSwingVLow;
    case stdAc::swingv_t::kLowest:  return kOsaka112AcSwingVLowest;
    default:                        return kOsaka112AcSwingVOn;
  }
}

/// Convert a native fan speed into its stdAc equivalent.
/// @param[in] spd The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::fanspeed_t IROsaka112Ac::toCommonFanSpeed(const uint8_t spd) {
  switch (spd) {
    case kOsaka112AcFanHigh: return stdAc::fanspeed_t::kMax;
    case kOsaka112AcFanMed:  return stdAc::fanspeed_t::kMedium;
    case kOsaka112AcFanLow:  return stdAc::fanspeed_t::kLow;
    case kOsaka112AcFanMin:  return stdAc::fanspeed_t::kMin;
    default:               return stdAc::fanspeed_t::kAuto;
  }
}

/// Convert a native vertical swing postion to it's common equivalent.
/// @param[in] setting A native position to convert.
/// @return The common vertical swing position.
stdAc::swingv_t IROsaka112Ac::toCommonSwingV(const uint8_t setting) {
  switch (setting) {
    case kOsaka112AcSwingVOff:       return stdAc::swingv_t::kOff;
    default:                       return stdAc::swingv_t::kAuto;
  }
}
/// Convert the current internal state into its stdAc::state_t equivalent.
/// @param[in] prev Ptr to the previous state if required.
/// @return The stdAc equivalent of the native settings.
stdAc::state_t IROsaka112Ac::toCommon(const stdAc::state_t *prev) const {
  stdAc::state_t result{};
  // Start with the previous state if given it.
  if (prev != NULL) result = *prev;
  result.protocol = decode_type_t::Osaka112AC;
  result.model = getModel();
  result.quiet = getQuiet(result.quiet);
  // The rest only get updated if it is a "normal" message.
  if (_.MsgType == kOsaka112AcNormal) {
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
String IROsaka112Ac::toString(void) const {
  String result = "";
  result.reserve(220);  // Reserve some heap for the string to reduce fragging.
  Osaka_ac_remote_model_t model = getModel();
  result += addModelToString(decode_type_t::Osaka112AC, model, false);
  result += addIntToString(_.MsgType, D_STR_TYPE);
  switch (_.MsgType) {
    case kOsaka112AcNormal:
      result += addBoolToString(_.Power, kPowerStr);
      result += addModeToString(_.Mode, kOsaka112AcAuto, kOsaka112AcCool,
                                kOsaka112AcHeat, kOsaka112AcDry, kOsaka112AcFan);
      result += addTempFloatToString(getTemp());
      result += addFanToString(_.Fan, kOsaka112AcFanHigh, kOsaka112AcFanLow,
                               kOsaka112AcFanAuto, kOsaka112AcFanMin,
                               kOsaka112AcFanMed);
      result += addSwingVToString(_.SwingV, kOsaka112AcSwingVOff,
                                            kOsaka112AcSwingVHighest,
                                            kOsaka112AcSwingVHigh,
                                            0xFF,  // unused
                                            kOsaka112AcSwingVMiddle,
                                            0xFF,  // unused
                                            kOsaka112AcSwingVLow,
                                            kOsaka112AcSwingVLowest,
                                            kOsaka112AcSwingVOff,
                                            kOsaka112AcSwingVOn,  // Swing
                                            0xFF, 0xFF);  // Both Unused
      if (model != Osaka_ac_remote_model_t::GZ055BE1) {
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
    case kOsaka112AcSpecial:
      result += addBoolToString(_.Quiet, kQuietStr);
      break;
  }
  return result;
}

#if DECODE_Osaka112AC
/// @file
/// @note There is no `decodedecodeOsaka112Ac()`.
///   It's the same as `decodeMitsubishi112()`. A shared routine is used.
///   You can find it in: ir_Mitsubishi.cpp
#endif  // DECODE_Osaka112AC

#if SEND_Osaka96AC
/// Send a Osaka 96-bit A/C message.
/// Status: BETA / Untested on a real device working.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendOsaka96Ac(const unsigned char data[], const uint16_t nbytes,
                         const uint16_t repeat) {
  enableIROut(38);
  for (uint16_t r = 0; r <= repeat; r++) {
    // Header
    mark(kOsaka96AcHdrMark);
    space(kOsaka96AcHdrSpace);
    // Data
    for (uint16_t pos = 0; pos < nbytes; pos++) {
      uint8_t databyte = data[pos];
      for (uint8_t bits = 0; bits < 8; bits += 2) {
        mark(kOsaka96AcBitMark);
        space(kOsaka96AcBitSpaces[GETBITS8(databyte, 8 - 2, 2)]);
        databyte <<= 2;
      }
    }
    // Footer
    mark(kOsaka96AcBitMark);
    space(kOsaka96AcGap);
  }
}
#endif  // SEND_Osaka96AC

#if SEND_Osaka112AC
void IRsend::sendOsaka112Ac(const unsigned char data[], const uint16_t nbytes,
                          const uint16_t repeat) {
  sendGeneric(kOsaka112AcHdrMark, kOsaka112AcHdrSpace,
              kOsaka112AcBitMark, kOsaka112AcOneSpace,
              kOsaka112AcBitMark, kOsaka112AcZeroSpace,
              kOsaka112AcBitMark, kOsaka112AcGap,
              data, nbytes, 38000, false, repeat, 50);
}
#endif  // SEND_Osaka112AC

#if DECODE_Osaka96AC
/// Decode the supplied Osaka96Ac message.
/// Status: ALPHA / Experimental.
/// @param[in,out] results Ptr to the data to decode & where to store the result
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return True if it can decode it, false if it can't.
bool IRrecv::decodeOsaka96Ac(decode_results* results, uint16_t offset,
                           const uint16_t nbits, const bool strict) {
  if (results->rawlen < nbits + kHeader + kFooter - 1 + offset)
    return false;  // Message is smaller than we expected.
  if (strict && nbits != kOsaka96AcBits)
    return false;  // Not strictly a Osaka96AC message.
  uint8_t data = 0;
  // Header.
  if (!matchMark(results->rawbuf[offset++], kOsaka96AcHdrMark)) return false;
  if (!matchSpace(results->rawbuf[offset++], kOsaka96AcHdrSpace)) return false;
  // Data (2 bits at a time)
  for (uint16_t bits_so_far = 0; bits_so_far < nbits; bits_so_far += 2) {
    if (bits_so_far % 8)
      data <<= 2;  // Make space for the new data bits.
    else
      data = 0;
    if (!matchMark(results->rawbuf[offset++], kOsaka96AcBitMark)) return false;
    uint8_t value = 0;
    while (value < kOsaka96AcSpaceCount) {
      if (matchSpace(results->rawbuf[offset], kOsaka96AcBitSpaces[value])) {
        data += value;
        break;
      }
      value++;
    }
    if (value >= kOsaka96AcSpaceCount) return false;  // No matches.
    offset++;
    *(results->state + bits_so_far / 8) = data;
  }
  // Footer
  if (!matchMark(results->rawbuf[offset++], kOsaka96AcBitMark)) return false;
  if (offset < results->rawlen &&
    !matchAtLeast(results->rawbuf[offset], kOsaka96AcGap)) return false;
  // Success
  results->decode_type = Osaka96AC;
  results->bits = nbits;
  return true;
}
#endif  // DECODE_Osaka96AC
