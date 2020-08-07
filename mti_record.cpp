
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <ctime>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <list>
#include <string>

using namespace std;

struct StampedPacket {
    timespec ts;
    XsDataPacket packet;
};

class CallbackHandler : public XsCallback {
public:
    CallbackHandler(size_t maxBufferSize = 5)
            : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0) {
    }

    virtual ~CallbackHandler() throw() {
    }

    bool packetAvailable() const {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    //XsDataPacket getNextPacket()
    StampedPacket getNextPacket() {
        assert(packetAvailable());
        xsens::Lock locky(&m_mutex);
        auto oldestPacket = m_packetBuffer.front();
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    virtual void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet) {
        xsens::Lock locky(&m_mutex);
        assert(packet != nullptr);
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void) getNextPacket();

        timespec ts;
        timespec_get(&ts, TIME_UTC);
        m_packetBuffer.push_back({ts, *packet});
        ++m_numberOfPacketsInBuffer;
        assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
    }

private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    //list<XsDataPacket> m_packetBuffer;
    list<StampedPacket> m_packetBuffer;
};

void write_data(const StampedPacket &sp, ofstream &ofs) {
    const auto &p = sp.packet;
    if (p.containsCalibratedAcceleration()) {
        //if (p.containsRawData()) {
        ofs << sp.ts.tv_sec << '\t' << sp.ts.tv_nsec << '\t';
        XsVector acc = p.calibratedAcceleration();
        XsVector gyr = p.calibratedGyroscopeData();
        //XsVector acc = p.rawAcceleration();
        //XsVector gyr = p.rasGyroscopeData();
        uint32_t status = p.status();
        ofs << acc[0] << '\t' << acc[1] << '\t' << acc[2] << '\t';
        ofs << gyr[0] << '\t' << gyr[1] << '\t' << gyr[2] << '\t';
        ofs << status << std::endl;
    }
}

//--------------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <outfilename>" << std::endl;
        return 0;
    }
    cout << "Creating XsControl object..." << endl;
    XsControl *control = XsControl::construct();
    assert(control != nullptr);

    XsVersion version;
    xdaVersion(&version);
    cout << "Using XDA version: " << version.toString().toStdString() << endl;

    // Lambda function for error handling
    auto handleError = [=](string errorString) {
        control->destruct();
        cout << errorString << endl;
        cout << "Press [ENTER] to continue." << endl;
        cin.get();
        return -1;
    };

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    // Find an MTi device
    XsPortInfo mtPort;
    for (auto const &portInfo : portInfoArray) {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty())
        return handleError("No MTi device found. Aborting.");

    cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: "
         << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

    cout << "Opening port..." << endl;
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        return handleError("Could not open port. Aborting.");

    // Get the device object
    XsDevice *device = control->device(mtPort.deviceId());
    assert(device != nullptr);

    cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString()
         << " opened." << endl;

    // Create and attach callback handler to device
    CallbackHandler callback;
    device->addCallbackHandler(&callback);

#if 0
    // Put the device into configuration mode before configuring the device
    cout << "Putting device into configuration mode..." << endl;
    if (!device->gotoConfig())
        return handleError("Could not put device into configuration mode. Aborting.");

    cout << "Configuring the device..." << endl;
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

    if (device->deviceId().isImu())
    {
        configArray.push_back(XsOutputConfiguration(XDI_DeltaV, 0));
        configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, 0));
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));
    }
    else if (device->deviceId().isVru() || device->deviceId().isAhrs())
    {
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
    }
    else if (device->deviceId().isGnss())
    {
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
        configArray.push_back(XsOutputConfiguration(XDI_LatLon, 0));
        configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 0));
        configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 0));
    }
    else
    {
        return handleError("Unknown device while configuring. Aborting.");
    }

    if (!device->setOutputConfiguration(configArray))
        return handleError("Could not configure MTi device. Aborting.");
#endif

    cout << "Putting device into measurement mode..." << endl;
    if (!device->gotoMeasurement())
        return handleError("Could not put device into measurement mode. Aborting.");

    cout << "Starting recording..." << endl;
    if (!device->startRecording())
        return handleError("Failed to start recording. Aborting.");

    cout << "\nMain loop. Recording data for 10 seconds." << endl;
    cout << string(79, '-') << endl;

    ofstream ofs(argv[1]);
    while (true) {
        if (callback.packetAvailable()) {
            cout << setw(5) << fixed << setprecision(2);

            // Retrieve a packet
            auto sp = callback.getNextPacket();
            const auto &packet = sp.packet;
            write_data(sp, ofs);
            if (packet.containsOrientation()) {
                XsQuaternion quaternion = packet.orientationQuaternion();
                cout << "\r"
                     << "q0:" << quaternion.w()
                     << ", q1:" << quaternion.x()
                     << ", q2:" << quaternion.y()
                     << ", q3:" << quaternion.z();

                XsEuler euler = packet.orientationEuler();
                cout << " |Roll:" << euler.roll()
                     << ", Pitch:" << euler.pitch()
                     << ", Yaw:" << euler.yaw();
            }

            cout << flush;
        }
        XsTime::msleep(0);
    }
    cout << "\n" << string(79, '-') << "\n";
    cout << endl;

    cout << "Stopping recording..." << endl;
    if (!device->stopRecording())
        return handleError("Failed to stop recording. Aborting.");

    cout << "Closing port..." << endl;
    control->closePort(mtPort.portName().toStdString());

    cout << "Freeing XsControl object..." << endl;
    control->destruct();

    cout << "Successful exit." << endl;

    cout << "Press [ENTER] to continue." << endl;
    cin.get();

    return 0;
}
