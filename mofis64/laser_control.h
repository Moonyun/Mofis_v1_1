#pragma once
#include <string>
#include <iostream>
#include <Windows.h>
#include <stdio.h>
#include "LogWindows.h"

class LaserControl {
private:
    bool ConnectLaser();
    bool DisConnectLaser();
    std::string m_com_name;
    HANDLE m_comm;
    int m_width;
    int m_frequency;
    int m_intensity;
    LogWindows* m_log_window;

public:
    LaserControl(const std::string& inComName, int inWidth, int inFrequency, int inIntensity, LogWindows* pLogWindow);
    LaserControl();
    ~LaserControl();
    bool OpenLaser();
    bool CloseLaser();
    bool SetParas(int inWidth, int inFrequency, int inIntensity , std::string inComname);
    bool BConnected();
};

LaserControl::LaserControl(const std::string& inComName, int inWidth, int inFrequency, int inIntensity, LogWindows* pLogWindow) {
    m_log_window = pLogWindow;
    m_com_name = inComName;
    m_width = inWidth;
    m_frequency = inFrequency;
    m_intensity = inIntensity;
    OpenLaser();
    if (ConnectLaser()) {
        m_log_window->AddLog("connect laser %s success\n", m_com_name);
    }
    else {
        m_log_window->AddLog("connect laser %s failed\n", m_com_name);

    }
}
LaserControl::LaserControl() {

}

LaserControl::~LaserControl() {
    if (DisConnectLaser()) {
        m_log_window->AddLog("disconnect %s success\n", m_com_name);
    }
    else {
        m_log_window->AddLog("disconnect %s failed\n", m_com_name);
    }
}

bool LaserControl::ConnectLaser() {
    m_comm = CreateFileA(("\\\\.\\"+m_com_name).data(),                //port name
        GENERIC_READ | GENERIC_WRITE, //Read/Write
        0,                            // No Sharing
        NULL,                         // No Security
        OPEN_EXISTING,// Open existing port only
        0,            // Non Overlapped I/O
        NULL);        // Null for Comm Devices

    if (m_comm == INVALID_HANDLE_VALUE) {
        m_log_window->AddLog("Error in opening serial port\n");
        return false;
    }
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    //Status = GetCommState(hComm, &dcbSerialParams);

    dcbSerialParams.BaudRate = CBR_115200;  // Setting BaudRate = 115200
    dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
    dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
    dcbSerialParams.Parity = NOPARITY;  // Setting Parity = None
    //dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    //dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    if (!SetCommState(m_comm, &dcbSerialParams)) {
        m_log_window->AddLog("set DCB failed\n", m_com_name);
        return false;
    }
    return true;
}

bool LaserControl::DisConnectLaser() {
    CloseHandle(m_comm);
    return true;
}

bool LaserControl::BConnected() {
    return m_comm != INVALID_HANDLE_VALUE;
}

bool LaserControl::OpenLaser() {
    char data[100];
    unsigned int low, heigh;
    unsigned char cnt = 0;
    unsigned int dianya, dianliu;
    data[cnt++] = 0xaa;
    data[cnt++] = 0x55;
    heigh = (m_width + 100) / 5.0 + 0.5;
    low = (1000000000.0 / m_frequency) / 5.0 - heigh;

    dianya = m_intensity / 100.0 * (0x3ff - 0x01ff) + 0x01ff;
    //dianliu = ui->lineEdit_dianliu->text().toUInt();
    dianliu = 0x03ff;

    data[cnt++] = (low >> 24) & 0xff;
    data[cnt++] = (low >> 16) & 0xff;
    data[cnt++] = (low >> 8) & 0xff;
    data[cnt++] = (low >> 0) & 0xff;

    data[cnt++] = (heigh >> 24) & 0xff;
    data[cnt++] = (heigh >> 16) & 0xff;
    data[cnt++] = (heigh >> 8) & 0xff;
    data[cnt++] = (heigh >> 0) & 0xff;

    data[cnt++] = (dianya >> 24) & 0xff;
    data[cnt++] = (dianya >> 16) & 0xff;
    data[cnt++] = (dianya >> 8) & 0xff;
    data[cnt++] = (dianya >> 0) & 0xff;

    data[cnt++] = (dianliu >> 24) & 0xff;
    data[cnt++] = (dianliu >> 16) & 0xff;
    data[cnt++] = (dianliu >> 8) & 0xff;
    data[cnt++] = (dianliu >> 0) & 0xff;

    data[cnt++] = 0;
    data[cnt++] = 0;
    data[cnt++] = 0;
    data[cnt++] = 0x01;
    data[cnt++] = 0x55;
    data[cnt++] = 0xaa;
    data[cnt++] = 0x0D;
    data[cnt++] = 0x0A;



    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    DWORD dNoOFBytestoWrite = cnt;

    auto Status = WriteFile(m_comm,        // Handle to the Serial port
        data,     // Data to be written to the port
        dNoOFBytestoWrite,  //No of bytes to write
        &dNoOfBytesWritten, //Bytes written
        NULL);
    m_log_window->AddLog("open laser\n");
    return Status;
}

bool LaserControl::CloseLaser() {
    char data[100];
    unsigned int low, heigh;
    unsigned char cnt = 0;
    unsigned int dianya, dianliu;
    data[cnt++] = 0xaa;
    data[cnt++] = 0x55;
    heigh = (m_width + 100) / 5.0 + 0.5;
    low = (1000000000.0 / m_frequency) / 5.0 - heigh;

    dianya = m_intensity / 100.0 * (0x3ff - 0x01ff) + 0x01ff;
    //dianliu = ui->lineEdit_dianliu->text().toUInt();
    dianliu = 0x03ff;

    data[cnt++] = (low >> 24) & 0xff;
    data[cnt++] = (low >> 16) & 0xff;
    data[cnt++] = (low >> 8) & 0xff;
    data[cnt++] = (low >> 0) & 0xff;

    data[cnt++] = (heigh >> 24) & 0xff;
    data[cnt++] = (heigh >> 16) & 0xff;
    data[cnt++] = (heigh >> 8) & 0xff;
    data[cnt++] = (heigh >> 0) & 0xff;

    data[cnt++] = (dianya >> 24) & 0xff;
    data[cnt++] = (dianya >> 16) & 0xff;
    data[cnt++] = (dianya >> 8) & 0xff;
    data[cnt++] = (dianya >> 0) & 0xff;

    data[cnt++] = (dianliu >> 24) & 0xff;
    data[cnt++] = (dianliu >> 16) & 0xff;
    data[cnt++] = (dianliu >> 8) & 0xff;
    data[cnt++] = (dianliu >> 0) & 0xff;

    data[cnt++] = 0;
    data[cnt++] = 0;
    data[cnt++] = 0;
    data[cnt++] = 0x00;
    data[cnt++] = 0x55;
    data[cnt++] = 0xaa;
    data[cnt++] = 0x0D;
    data[cnt++] = 0x0A;

    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    DWORD dNoOFBytestoWrite = cnt;

    auto Status = WriteFile(m_comm,        // Handle to the Serial port
        data,     // Data to be written to the port
        dNoOFBytestoWrite,  //No of bytes to write
        &dNoOfBytesWritten, //Bytes written
        NULL);
    m_log_window->AddLog("close laser \n");
    return Status;
}

bool LaserControl::SetParas(int inWidth, int inFrequency, int inIntensity, std::string inComname) {
    m_log_window->AddLog("laser paras : before : %d %d %d %s\n", m_width, m_frequency, m_intensity,m_com_name);
    m_width = inWidth;
    m_frequency = inFrequency;
    m_intensity = inIntensity;
    m_com_name = inComname;
    m_log_window->AddLog("laser paras : after  : %d %d %d %s\n", m_width, m_frequency, m_intensity,m_com_name);
    return true;
}

