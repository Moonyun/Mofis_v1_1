#ifndef NATIVEUTIL_H
#define NATIVEUTIL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>
#include <process.h>
#include "GenICam/CAPI/SDK.h"

static void displayDeviceInfo(GENICAM_Camera* pCameraList, int cameraCnt) {
	GENICAM_Camera* pDisplayCamera = NULL;
	GENICAM_GigECameraInfo GigECameraInfo;
	GENICAM_GigECamera* pGigECamera = NULL;
	int cameraIndex;
	int keyType;
	const char* vendorName = NULL;
	char vendorNameCat[11];
	const char* deviceUserID = NULL;
	char deviceUserIDCat[16];
	const char* ipAddress = NULL;
	int ret = 0;
	// 打印Title行     
	// Print title line     
	//printf("\nIdx Type Vendor     Model      S/N             DeviceUserID    IP Address    \n");
	//printf("------------------------------------------------------------------------------\n");
	for (cameraIndex = 0; cameraIndex < cameraCnt; cameraIndex++) {
		pDisplayCamera = &pCameraList[cameraIndex];
		// 设备列表的相机索引  最大表示字数：3        
		// Camera index in device list, display in 3 characters        
		//printf("%-3d", cameraIndex + 1);
		// 相机的设备类型（GigE，U3V，CL，PCIe）        
		// Camera type         
		keyType = pDisplayCamera->getType(pDisplayCamera);
		switch (keyType) {
		case typeGige:
			//printf(" GigE");
			break;
		case typeUsb3:
			//printf(" U3V ");
			break;
		case typeCL:
			//printf(" CL  ");
			break;
		case typePCIe:
			//printf(" PCIe");
			break;
		default:
			//printf("     ");
			break;
		}
		// 制造商信息  最大表示字数：10        
		// Camera vendor name, display in 10 characters        
		vendorName = pDisplayCamera->getVendorName(pDisplayCamera);
		if (strlen(vendorName) > 10) {
			strncpy(vendorNameCat, vendorName, 7);
			vendorNameCat[7] = '\0';
			strcat(vendorNameCat, "...");
			//printf(" %-10.10s", vendorNameCat);
		}
		else {
			//printf(" %-10.10s", vendorName);
		}
		// 相机的型号信息 最大表示字数：10  
		// Camera model name, display in 10 characters      
		printf(" %-10.10s", pDisplayCamera->getModelName(pDisplayCamera));
		// 相机的序列号 最大表示字数：15    
		// Camera serial number, display in 15 characters    
		printf(" %-15.15s", pDisplayCamera->getSerialNumber(pDisplayCamera));
		// 自定义用户ID 最大表示字数：15       
		// Camera user id, display in 15 characters  
		deviceUserID = pDisplayCamera->getName(pDisplayCamera);

		if (strlen(deviceUserID) > 15) {
			strncpy(deviceUserIDCat, deviceUserID, 12);
			deviceUserIDCat[12] = '\0';
			strcat(deviceUserIDCat, "...");
			//printf(" %-15.15s", deviceUserIDCat);
		}
		else {
			memcpy(deviceUserIDCat, deviceUserID, sizeof(deviceUserIDCat));
			//printf(" %-15.15s", deviceUserIDCat);
		}
		// GigE相机时获取IP地址  
		// IP address of GigE camera       
		if (keyType == typeGige) {
			GigECameraInfo.pCamera = pDisplayCamera;
			ret = GENICAM_createGigECamera(&GigECameraInfo, &pGigECamera);
			if (ret == 0) {
				ipAddress = pGigECamera->getIpAddress(pGigECamera);
				if (ipAddress != NULL) {
					//printf(" %s", pGigECamera->getIpAddress(pGigECamera));
				}
			}
		}
		//printf("\n");
	}
	return;
}

static int GENICAM_connect(GENICAM_Camera* pGetCamera) {
	int isConnectSuccess;
	isConnectSuccess = pGetCamera->connect(pGetCamera, accessPermissionControl);
	if (isConnectSuccess != 0) {
		//printf("connect camera failed.\n");
		return -1;
	}
	return 0;
}

static char* trim(char* pStr) {
	char* pDst = pStr;
	char* pTemStr = NULL;
	int ret = -1;
	if (pDst != NULL) {
		pTemStr = pDst + strlen(pStr) - 1;
		while (*pDst == ' ') {
			pDst++;
		}
		while ((pTemStr > pDst) && (*pTemStr == ' ')) {
			*pTemStr-- = '\0';
		}
	}
	return pDst;
}

static int isInputValid(char* pInpuStr) {
	char numChar;
	char* pStr = pInpuStr;
	while (*pStr != '\0') {
		numChar = *pStr;
		if ((numChar > '9') || (numChar < '0')) {
			return -1;
		}
		pStr++;
	}
	return 0;
}

static int selectDevice(int cameraCnt) {
	char inputStr[256] = { 0 };
	char* pTrimStr;
	int inputIndex = -1;
	int ret = -1;
	//printf("\nPlease input the camera index: ");
	while (1) {
		memset(inputStr, 0, sizeof(inputStr));
		//scanf("%s", inputStr);
		fflush(stdin);
		pTrimStr = trim(inputStr);
		ret = isInputValid(pTrimStr);
		if (ret == 0) {
			inputIndex = atoi(pTrimStr);
			// 输入的序号从1开始            
			// Input index starts from 1         
			inputIndex -= 1;
			if ((inputIndex >= 0) && (inputIndex < cameraCnt)) {
				break;
			}
		}
		//printf("Input invalid! Please input the camera index: ");
	}
	return inputIndex;
}
static int modifyCameraHeight(GENICAM_Camera* pGetCamera, int inHeight) {
	int isIntNodeSuccess;
	int64_t HeightValue = 0;
	GENICAM_IntNode* pIntNode = NULL;
	GENICAM_IntNodeInfo intNodeInfo = { 0 };
	intNodeInfo.pCamera = pGetCamera;
	memcpy(intNodeInfo.attrName, "Height", sizeof("Height"));
	isIntNodeSuccess = GENICAM_createIntNode(&intNodeInfo, &pIntNode);
	if (0 != isIntNodeSuccess) {
		//printf("GENICAM_createIntNode fail.\n");
		return -1;
	}
	isIntNodeSuccess = pIntNode->getValue(pIntNode, &HeightValue);
	if (0 != isIntNodeSuccess != 0) {

		//printf("get Height fail.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	else {
		//printf("before change ,Height is %d\n", HeightValue);
	}
	isIntNodeSuccess = pIntNode->setValue(pIntNode, inHeight);
	if (0 != isIntNodeSuccess) {
		//printf("set Height fail ydc.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	isIntNodeSuccess = pIntNode->getValue(pIntNode, &HeightValue);
	if (0 != isIntNodeSuccess) {
		printf("get Height fail.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	else {
		//printf("after change ,Height is %d\n", HeightValue);
		pIntNode->release(pIntNode);
	}
	return 0;
}


static int modifyCameraOffsetX(GENICAM_Camera* pGetCamera, int inOffsetX) {
	int isIntNodeSuccess;
	int64_t OffsetXValue = 0;
	GENICAM_IntNode* pIntNode = NULL;
	GENICAM_IntNodeInfo intNodeInfo = { 0 };
	intNodeInfo.pCamera = pGetCamera;
	memcpy(intNodeInfo.attrName, "OffsetX", sizeof("OffsetX"));
	isIntNodeSuccess = GENICAM_createIntNode(&intNodeInfo, &pIntNode);
	if (0 != isIntNodeSuccess) {
		//printf("GENICAM_createIntNode fail.\n");
		return -1;
	}
	isIntNodeSuccess = pIntNode->getValue(pIntNode, &OffsetXValue);
	if (0 != isIntNodeSuccess != 0) {

		printf("get OffsetX fail.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	else {
		//printf("before change ,OffsetX is %d\n", OffsetXValue);
	}
	isIntNodeSuccess = pIntNode->setValue(pIntNode, inOffsetX);
	if (0 != isIntNodeSuccess) {
		//printf("set OffsetX fail ydc.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	isIntNodeSuccess = pIntNode->getValue(pIntNode, &OffsetXValue);
	if (0 != isIntNodeSuccess) {
		//printf("get OffsetX fail.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	else {
		//printf("after change ,OffsetX is %d\n", OffsetXValue);
		pIntNode->release(pIntNode);
	}
	return 0;
}

static int modifyCameraWidth(GENICAM_Camera* pGetCamera, int64_t inWidth) {
	int isIntNodeSuccess;
	int64_t widthValue = 0;
	GENICAM_IntNode* pIntNode = NULL;
	GENICAM_IntNodeInfo intNodeInfo = { 0 };
	intNodeInfo.pCamera = pGetCamera;
	memcpy(intNodeInfo.attrName, "Width", sizeof("Width"));
	isIntNodeSuccess = GENICAM_createIntNode(&intNodeInfo, &pIntNode);
	if (0 != isIntNodeSuccess) {
		//printf("GENICAM_createIntNode fail.\n");
		return -1;
	}
	isIntNodeSuccess = pIntNode->getValue(pIntNode, &widthValue);
	if (0 != isIntNodeSuccess != 0) {

		//printf("get Width fail.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	else {
		//printf("before change ,Width is %d\n", widthValue);
		//pIntNode->release(pIntNode);
	}
	isIntNodeSuccess = pIntNode->setValue(pIntNode, inWidth);
	if (0 != isIntNodeSuccess) {
		//printf("set Width fail ydc.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	isIntNodeSuccess = pIntNode->getValue(pIntNode, &widthValue);
	if (0 != isIntNodeSuccess) {
		//printf("get Width fail.\n");
		pIntNode->release(pIntNode);
		return -1;
	}
	else {
		//printf("after change ,Width is %d\n", widthValue);
		pIntNode->release(pIntNode);
	}
	return 0;
}
static int modifyCameraAcquisitionFrameRate(GENICAM_Camera* pGetCamera, double inAcquisitionFrameRate) {
	int isDoubleNodeSuccess;
	double AcquisitionFrameRateValue = 0.0;
	GENICAM_DoubleNode* pDoubleNode = NULL;
	GENICAM_DoubleNodeInfo doubleNodeInfo = { 0 };
	doubleNodeInfo.pCamera = pGetCamera;
	memcpy(doubleNodeInfo.attrName, "AcquisitionFrameRate", sizeof("AcquisitionFrameRate"));
	isDoubleNodeSuccess = GENICAM_createDoubleNode(&doubleNodeInfo, &pDoubleNode);
	if (0 != isDoubleNodeSuccess) {
		//printf("GENICAM_createIntNode fail.\n");
		return -1;
	}
	isDoubleNodeSuccess = pDoubleNode->getValue(pDoubleNode, &AcquisitionFrameRateValue);
	if (0 != isDoubleNodeSuccess != 0) {

		//printf("get AcquisitionFrameRate fail.\n");
		pDoubleNode->release(pDoubleNode);
		return -1;
	}
	else {
		//printf("before change ,AcquisitionFrameRate is %f\n", AcquisitionFrameRateValue);
	}
	isDoubleNodeSuccess = pDoubleNode->setValue(pDoubleNode, inAcquisitionFrameRate);
	if (0 != isDoubleNodeSuccess) {
		//printf("set AcquisitionFrameRate fail ydc.\n");
		pDoubleNode->release(pDoubleNode);
		return -1;
	}
	isDoubleNodeSuccess = pDoubleNode->getValue(pDoubleNode, &AcquisitionFrameRateValue);
	if (0 != isDoubleNodeSuccess) {
		//printf("get AcquisitionFrameRate fail.\n");
		pDoubleNode->release(pDoubleNode);
		return -1;
	}
	else {
		//printf("after change ,AcquisitionFrameRate is %f\n", AcquisitionFrameRateValue);
		pDoubleNode->release(pDoubleNode);
	}
	return 0;
}

static int modifyCameraExposureTime(GENICAM_Camera* pGetCamera, double inExposureTime) {
	int isDoubleNodeSuccess;
	double exposureTimeValue = 0.0;
	GENICAM_DoubleNode* pDoubleNode = NULL;
	GENICAM_DoubleNodeInfo doubleNodeInfo = { 0 };
	doubleNodeInfo.pCamera = pGetCamera;
	memcpy(doubleNodeInfo.attrName, "ExposureTime", sizeof("ExposureTime"));
	isDoubleNodeSuccess = GENICAM_createDoubleNode(&doubleNodeInfo, &pDoubleNode);
	if (0 != isDoubleNodeSuccess) {
		//printf("GENICAM_createIntNode fail.\n");
		return -1;
	}
	isDoubleNodeSuccess = pDoubleNode->getValue(pDoubleNode, &exposureTimeValue);
	if (0 != isDoubleNodeSuccess != 0) {

		//printf("get exposureTime fail.\n");
		pDoubleNode->release(pDoubleNode);
		return -1;
	}
	else {
		//printf("before change ,exposureTime is %f\n", exposureTimeValue);
	}
	isDoubleNodeSuccess = pDoubleNode->setValue(pDoubleNode, inExposureTime);
	if (0 != isDoubleNodeSuccess) {
		//printf("set exposureTime fail ydc.\n");
		pDoubleNode->release(pDoubleNode);
		return -1;
	}
	isDoubleNodeSuccess = pDoubleNode->getValue(pDoubleNode, &exposureTimeValue);
	if (0 != isDoubleNodeSuccess) {
		//printf("get exposureTime fail.\n");
		pDoubleNode->release(pDoubleNode);
		return -1;
	}
	else {
		//printf("after change ,exposureTime is %f\n", exposureTimeValue);
		pDoubleNode->release(pDoubleNode);
	}
	return 0;
}
static int GENICAM_CreateStreamSource(GENICAM_Camera* pGetCamera, GENICAM_StreamSource** ppStreamSource) {
	int isCreateStreamSource;
	GENICAM_StreamSourceInfo stStreamSourceInfo;
	stStreamSourceInfo.channelId = 0;
	stStreamSourceInfo.pCamera = pGetCamera;
	isCreateStreamSource = GENICAM_createStreamSource(&stStreamSourceInfo, ppStreamSource);
	if (isCreateStreamSource != 0) {
		//printf("create stream obj  fail.\r\n");
		return -1;
	}
	return 0;
}

static int GENICAM_startGrabbing(GENICAM_StreamSource* pStreamSource) {
	int isStartGrabbingSuccess;
	GENICAM_EGrabStrategy eGrabStrategy;
	eGrabStrategy = grabStrartegySequential;
	//eGrabStrategy = grabStrartegyLatestImage;
	isStartGrabbingSuccess = pStreamSource->startGrabbing(pStreamSource, 0, eGrabStrategy);
	if (isStartGrabbingSuccess != 0) {
		//printf("StartGrabbing  fail.\n");
		return -1;
	}
	return 0;
}
static int GENICAM_stopGrabbing(GENICAM_StreamSource* pStreamSource) {
	int isStopGrabbingSuccess;
	isStopGrabbingSuccess = pStreamSource->stopGrabbing(pStreamSource);
	if (isStopGrabbingSuccess != 0) {
		//printf("StopGrabbing  fail.\n");
		return -1;
	}
	return 0;
}

#endif