#include "FPGA_Mini.h"
#include "IConnection.h"
#include "Logger.h"
#include "LMS64CProtocol.h"
#include <ciso646>
#include <vector>
#include <map>
#include <math.h>

namespace lime
{

FPGA_Mini::FPGA_Mini() : FPGA(){}
/** @brief Configures FPGA PLLs to LimeLight interface frequency
*/
int FPGA_Mini::SetInterfaceFreq(double txRate_Hz, double rxRate_Hz, double txPhase, double rxPhase, int channel)
{
    int status = 0;

    FPGA_PLL_clock clocks[4];

    clocks[0].bypass = false;
    clocks[0].index = 0;
    clocks[0].outFrequency = txRate_Hz;
    clocks[0].phaseShift_deg = 0;
    clocks[0].findPhase = false;
    clocks[1].bypass = false;
    clocks[1].index = 1;
    clocks[1].outFrequency = txRate_Hz;
    clocks[1].findPhase = false;
    clocks[1].phaseShift_deg = txPhase;
    clocks[2].bypass = false;
    clocks[2].index = 2;
    clocks[2].outFrequency = rxRate_Hz;
    clocks[2].phaseShift_deg = 0;
    clocks[2].findPhase = false;
    clocks[3].bypass = false;
    clocks[3].index = 3;
    clocks[3].outFrequency = rxRate_Hz;
    clocks[3].findPhase = false;
    clocks[3].phaseShift_deg = rxPhase;

    status = SetPllFrequency(0, rxRate_Hz, clocks, 4);

    return status;
}

/** @brief Configures FPGA PLLs to LimeLight interface frequency
*/
int FPGA_Mini::SetInterfaceFreq(double txRate_Hz, double rxRate_Hz, int channel)
{
    int status = 0;
    uint32_t reg20;
    const double rxPhC1[] = { 91.08, 89.46 };
    const double rxPhC2[] = { -1 / 6e6, 1.24e-6 };
    const double txPhC1[] = { 89.75, 89.61 };
    const double txPhC2[] = { -3.0e-7, 2.71e-7 };

    const std::vector<uint32_t> spiAddr = { 0x0021, 0x0022, 0x0023, 0x0024,
        0x0027, 0x002A, 0x0400, 0x040C,
        0x040B, 0x0400, 0x040B, 0x0400 };
    const int bakRegCnt = spiAddr.size() - 4;
    
    std::vector<uint32_t> dataWr;
    dataWr.push_back(uint32_t(0x002F) << 16);
    uint32_t chipVersion=0;
    connection->ReadLMS7002MSPI(dataWr.data(), &chipVersion, 1, channel);
    dataWr.clear(); 
    
    bool phaseSearch = false;
      if (txRate_Hz >= 5e6 || rxRate_Hz >= 5e6)
        phaseSearch = true;

    std::vector<uint32_t> dataRd;

    if (phaseSearch)
    {
        dataWr.resize(spiAddr.size());
        dataRd.resize(spiAddr.size());
        //backup registers
        dataWr[0] = (uint32_t(0x0020) << 16);
        connection->ReadLMS7002MSPI(dataWr.data(), &reg20, 1, channel);

        dataWr[0] = (1 << 31) | (uint32_t(0x0020) << 16) | 0xFFFD; //msbit 1=SPI write
        connection->WriteLMS7002MSPI(dataWr.data(), 1, channel);

        for (int i = 0; i < bakRegCnt; ++i)
            dataWr[i] = (spiAddr[i] << 16);
         connection->ReadLMS7002MSPI(dataWr.data(),dataRd.data(), bakRegCnt, channel);
    }

    if ((txRate_Hz >= 5e6) && (rxRate_Hz >= 5e6))
    {
        FPGA_PLL_clock clocks[4];
        clocks[2].bypass = false;
        clocks[2].index = 0;
        clocks[2].outFrequency = txRate_Hz;
        clocks[2].phaseShift_deg = 0;
        clocks[2].findPhase = false;
        clocks[3].bypass = false;
        clocks[3].index = 1;
        clocks[3].outFrequency = txRate_Hz;
        clocks[3].findPhase = false;
        if (chipVersion == 0x3841)
            clocks[3].phaseShift_deg = txPhC1[1] + txPhC2[1] * txRate_Hz;
        else
            clocks[3].phaseShift_deg = txPhC1[0] + txPhC2[0] * txRate_Hz;
        clocks[0].bypass = false;
        clocks[0].index = 2;
        clocks[0].outFrequency = rxRate_Hz;
        clocks[0].phaseShift_deg = 0;
        clocks[0].findPhase = false;
        clocks[1].bypass = false;
        clocks[1].index = 3;
        clocks[1].outFrequency = rxRate_Hz;
        clocks[1].findPhase = false;
        if (chipVersion == 0x3841)
            clocks[1].phaseShift_deg = rxPhC1[1] + rxPhC2[1] * rxRate_Hz;
        else
            clocks[1].phaseShift_deg = rxPhC1[0] + rxPhC2[0] * rxRate_Hz;

        if (phaseSearch)
        {
            {
                clocks[1].findPhase = true;
                const std::vector<uint32_t> spiData = { 0x0E9F, 0x07FF, 0x5550, 0xE4E4,
                    0xE4E4, 0x0086, 0x028D, 0x00FF, 0x5555, 0x02CD, 0xAAAA, 0x02ED };
                //Load test config
                const int setRegCnt = spiData.size();
                for (int i = 0; i < setRegCnt; ++i)
                    dataWr[i] = (1 << 31) | (uint32_t(spiAddr[i]) << 16) | spiData[i]; //msbit 1=SPI write
                connection->WriteLMS7002MSPI(dataWr.data(), setRegCnt, channel);
                status = SetPllFrequency(0, rxRate_Hz, clocks, 4);
            }
            {
                clocks[1].findPhase = false;
                const std::vector<uint32_t> spiData = { 0x0E9F, 0x07FF, 0x5550, 0xE4E4, 0xE4E4, 0x0484 };
                connection->WriteRegister(0x000A, 0x0000);
                //Load test config
                const int setRegCnt = spiData.size();
                for (int i = 0; i < setRegCnt; ++i)
                    dataWr[i] = (1 << 31) | (uint32_t(spiAddr[i]) << 16) | spiData[i]; //msbit 1=SPI write
                connection->WriteLMS7002MSPI(dataWr.data(), setRegCnt, channel);
                clocks[3].findPhase = true;
                connection->WriteRegister(0x000A, 0x0200);

            }
        }
        status = SetPllFrequency(0, rxRate_Hz, clocks, 4);
    }
    else
    {
        status = SetDirectClocking(0);
        if (status == 0)
            status = SetDirectClocking(1);
    }

    if (phaseSearch)
    {
        //Restore registers
        for (int i = 0; i < bakRegCnt; ++i)
            dataWr[i] = (1 << 31) | (uint32_t(spiAddr[i]) << 16) | dataRd[i]; //msbit 1=SPI write
        connection->WriteLMS7002MSPI(dataWr.data(), bakRegCnt, channel);
        dataWr[0] = (1 << 31) | (uint32_t(0x0020) << 16) | reg20; //msbit 1=SPI write
        connection->WriteLMS7002MSPI(dataWr.data(), 1, channel);
        connection->WriteRegister(0x000A, 0);
    }
    return status;
}

int FPGA_Mini::UploadWFM(const void* const* samples, uint8_t chCount, size_t sample_count, StreamConfig::StreamDataFormat format, int epIndex)
{
   return ReportError("UploadWFM not supported on LimeSDR-Mini"); 
}


int FPGA_Mini::ReadRawStreamData(char* buffer, unsigned length, int epIndex, int timeout_ms)
{
    int totalBytesReceived = 0;
    StopStreaming();

    //ResetStreamBuffers();
    connection->WriteRegister(0x0008, 0x0100 | 0x2);
    connection->WriteRegister(0x0007, 1);

    StartStreaming();

    int handle = connection->BeginDataReading(buffer, length, 0);
    if (connection->WaitForReading(handle, timeout_ms))
        totalBytesReceived = connection->FinishDataReading(buffer, length, handle);

    connection->AbortReading(0);
    StopStreaming();

    return totalBytesReceived;
}



} //namespace lime