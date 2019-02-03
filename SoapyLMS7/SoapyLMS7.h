/**
@file	SoapyLMS7.h
@brief	Header for Soapy SDR + IConnection bindings.
@author Lime Microsystems (www.limemicro.com)
*/

#include <SoapySDR/Device.hpp>
#include <ConnectionRegistry.h>
#include <mutex>
#include <chrono>
#include <map>
#include <set>
#include "Streamer.h"

static const double DEFAULT_CLOCK_RATE = 80e6;

namespace lime
{
    class LIME_API LMS7_Device;
}
struct IConnectionStream;

class SoapyLMS7 : public SoapySDR::Device
{
public:
    SoapyLMS7(const lime::ConnectionHandle &handle, const SoapySDR::Kwargs &args);

    ~SoapyLMS7(void);

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const override;

    std::string getHardwareKey(void) const override;

    SoapySDR::Kwargs getHardwareInfo(void) const override;

    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels(const int direction) const override;

    bool getFullDuplex(const int direction, const size_t channel) const override;

    /*******************************************************************
     * Stream API
     ******************************************************************/
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const override;

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const override;

    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const override;

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels = std::vector<size_t>(),
        const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

    void closeStream(SoapySDR::Stream *stream) override;

    size_t getStreamMTU(SoapySDR::Stream *stream) const override;

    int activateStream(
        SoapySDR::Stream *stream,
        const int flags = 0,
        const long long timeNs = 0,
        const size_t numElems = 0) override;

    int deactivateStream(
        SoapySDR::Stream *stream,
        const int flags = 0,
        const long long timeNs = 0) override;

    int readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs = 100000) override;

    int _readStreamAligned(
        IConnectionStream *stream,
        char * const *buffs,
        size_t numElems,
        uint64_t requestTime,
        lime::StreamChannel::Metadata &mdOut,
        const long timeoutMs);

    int writeStream(
        SoapySDR::Stream *stream,
        const void * const *buffs,
        const size_t numElems,
        int &flags,
        const long long timeNs = 0,
        const long timeoutUs = 100000) override;

    int readStreamStatus(
        SoapySDR::Stream *stream,
        size_t &chanMask,
        int &flags,
        long long &timeNs,
        const long timeoutUs = 100000) override;

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const override;

    void setAntenna(const int direction, const size_t channel, const std::string &name) override;

    std::string getAntenna(const int direction, const size_t channel) const override;

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    bool hasDCOffsetMode(const int direction, const size_t channel) const override;

    void setDCOffsetMode(const int direction, const size_t channel, const bool automatic) override;

    bool getDCOffsetMode(const int direction, const size_t channel) const override;

    bool hasDCOffset(const int direction, const size_t channel) const override;

    void setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset) override;

    std::complex<double> getDCOffset(const int direction, const size_t channel) const override;

    bool hasIQBalance(const int direction, const size_t channel) const override;

    void setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance) override;

    std::complex<double> getIQBalance(const int direction, const size_t channel) const override;

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const override;

    void setGain(const int direction, const size_t channel, const double value) override;

    double getGain(const int direction, const size_t channel) const override;

    void setGain(const int direction, const size_t channel, const std::string &name, const double value) override;

    double getGain(const int direction, const size_t channel, const std::string &name) const override;

    SoapySDR::Range getGainRange(const int direction, const size_t channel) const override;

    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const override;

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const override;

    void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

    void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

    double getFrequency(const int direction, const size_t channel, const std::string &name) const override;

    double getFrequency(const int direction, const size_t channel) const override;

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const override;

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel) const override;

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const override;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    std::map<size_t, int> _interps;
    std::map<size_t, int> _decims;

    void setSampleRate(const int direction, const size_t channel, const double rate) override;

    double getSampleRate(const int direction, const size_t channel) const override;

    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;

    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const override;

    /*******************************************************************
     * Bandwidth API
     ******************************************************************/

    std::map<int, std::map<size_t, double>> _actualBw;

    void setBandwidth(const int direction, const size_t channel, const double bw) override;

    double getBandwidth(const int direction, const size_t channel) const override;

    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const override;

    /*******************************************************************
     * Clocking API
     ******************************************************************/

    void setMasterClockRate(const double rate) override;

    double getMasterClockRate(void) const override;

    SoapySDR::RangeList getMasterClockRates(void) const override;

    /*******************************************************************
     * Time API
     ******************************************************************/

    bool hasHardwareTime(const std::string &what = "") const override;

    long long getHardwareTime(const std::string &what = "") const override;

    void setHardwareTime(const long long timeNs, const std::string &what = "") override;

    /*******************************************************************
     * Sensor API
     ******************************************************************/

    std::vector<std::string> listSensors(void) const override;

    SoapySDR::ArgInfo getSensorInfo(const std::string &name) const override;

    std::string readSensor(const std::string &name) const override;

    std::vector<std::string> listSensors(const int direction, const size_t channel) const override;

    SoapySDR::ArgInfo getSensorInfo(const int direction, const size_t channel, const std::string &name) const override;

    std::string readSensor(const int direction, const size_t channel, const std::string &name) const override;

    /*******************************************************************
     * Register API
     ******************************************************************/

    std::vector<std::string> listRegisterInterfaces(void) const override;

    void writeRegister(const std::string &name, const unsigned addr, const unsigned value) override;

    unsigned readRegister(const std::string &name, const unsigned addr) const override;

    void writeRegister(const unsigned addr, const unsigned value) override;

    unsigned readRegister(const unsigned addr) const override;

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const override;

    void writeSetting(const std::string &key, const std::string &value) override;

    SoapySDR::ArgInfoList getSettingInfo(const int direction, const size_t channel) const override;

    void writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value) override;

    std::string readSetting(const std::string &key) const override;

    std::string readSetting(const int direction, const size_t channel, const std::string &key) const override;

    /*******************************************************************
     * GPIO API
     ******************************************************************/

    std::vector<std::string> listGPIOBanks(void) const override;

    void writeGPIO(const std::string &bank, const unsigned value) override;

    unsigned readGPIO(const std::string &bank) const override;

    void writeGPIODir(const std::string &bank, const unsigned dir) override;

    unsigned readGPIODir(const std::string &bank) const override;

private:
    const SoapySDR::Kwargs _deviceArgs; //!< stash of constructor arguments
    const std::string _moduleName;
    lime::LMS7_Device * lms7Device;
    double sampleRate;
    std::set<std::pair<int, size_t>> _channelsToCal;
    mutable std::recursive_mutex _accessMutex;
};
