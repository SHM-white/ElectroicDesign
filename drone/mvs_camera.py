"""OpenCV-compatible capture wrapper for Hikrobot MVS cameras."""

import os
import platform
import sys
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof

import cv2
import numpy as np


_configured_root = os.environ.get("MVCAM_SDK_PATH", "/opt/MVS")
MVS_ROOT = (
    _configured_root
    if os.path.isfile(os.path.join(_configured_root, "include", "MvCameraControl.h"))
    else "/opt/MVS"
)
MVS_PYTHON = os.path.join(MVS_ROOT, "Samples", "64", "Python", "MvImport")


def _prepare_mvs_environment() -> None:
    architecture = platform.machine()
    library_dir = "64" if architecture == "x86_64" else "aarch64"
    mvs_lib = os.path.join(MVS_ROOT, "lib")
    native_lib = os.path.join(mvs_lib, library_dir)

    os.environ["MVCAM_SDK_PATH"] = MVS_ROOT
    os.environ["MVCAM_COMMON_RUNENV"] = mvs_lib
    os.environ["MVCAM_SOFTWARE_LIBENV"] = mvs_lib
    os.environ["MVCAM_GENICAM_CLPROTOCOL"] = os.path.join(mvs_lib, "CLProtocol")
    os.environ["LD_LIBRARY_PATH"] = native_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")

    if MVS_PYTHON not in sys.path:
        sys.path.insert(0, MVS_PYTHON)


_prepare_mvs_environment()

try:
    from MvCameraControl_class import (  # type: ignore[import-not-found]
        MV_ACCESS_Exclusive,
        MV_CC_DEVICE_INFO,
        MV_CC_DEVICE_INFO_LIST,
        MV_CC_PIXEL_CONVERT_PARAM_EX,
        MV_FRAME_OUT,
        MV_GIGE_DEVICE,
        MV_TRIGGER_MODE_OFF,
        MV_USB_DEVICE,
        MvCamera,
        PixelType_Gvsp_RGB8_Packed,
    )
except ImportError as exc:
    raise ImportError(
        f"MVS Python SDK not found under {MVS_PYTHON}; install the full MVS SDK"
    ) from exc


class MvsCapture:
    """Provide ``read``/``release`` methods compatible with VideoCapture."""

    def __init__(self, device_id: int = 0, timeout_ms: int = 1000,
                 auto_exposure: bool = False,
                 exposure_us: float = 100000.0,
                 gain: float = 8.0):
        self.device_id = device_id
        self.timeout_ms = timeout_ms
        self.auto_exposure = auto_exposure
        self.exposure_us = exposure_us
        self.gain = gain
        self.camera = MvCamera()
        self._opened = False
        self._grabbing = False
        self.width = 0
        self.height = 0
        self.model = ""
        self.serial = ""
        self._open()

    @staticmethod
    def _decode(value) -> str:
        data = memoryview(value).tobytes().split(b"\x00", 1)[0]
        return data.decode("utf-8", errors="replace")

    def _open(self) -> None:
        ret = MvCamera.MV_CC_Initialize()
        if ret != 0:
            raise RuntimeError(f"MVS initialize failed: 0x{ret:08x}")

        device_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
        if ret != 0:
            MvCamera.MV_CC_Finalize()
            raise RuntimeError(f"MVS device enumeration failed: 0x{ret:08x}")
        if device_list.nDeviceNum == 0:
            MvCamera.MV_CC_Finalize()
            raise RuntimeError("MVS found no cameras")
        if not 0 <= self.device_id < device_list.nDeviceNum:
            MvCamera.MV_CC_Finalize()
            raise RuntimeError(
                f"MVS camera index {self.device_id} is out of range; "
                f"found {device_list.nDeviceNum} camera(s)"
            )

        device_info = cast(
            device_list.pDeviceInfo[self.device_id], POINTER(MV_CC_DEVICE_INFO)
        ).contents
        if device_info.nTLayerType == MV_USB_DEVICE:
            self.model = self._decode(device_info.SpecialInfo.stUsb3VInfo.chModelName)
            self.serial = self._decode(device_info.SpecialInfo.stUsb3VInfo.chSerialNumber)

        ret = self.camera.MV_CC_CreateHandle(device_info)
        if ret != 0:
            MvCamera.MV_CC_Finalize()
            raise RuntimeError(f"MVS create handle failed: 0x{ret:08x}")

        try:
            ret = self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                raise RuntimeError(f"MVS open camera failed: 0x{ret:08x}")
            self._opened = True

            ret = self.camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                raise RuntimeError(f"MVS disable trigger failed: 0x{ret:08x}")

            if self.auto_exposure:
                self._enable_auto_image_settings()
            else:
                self._set_manual_image_settings()

            # A failed quality setting is non-fatal on cameras without this node.
            self.camera.MV_CC_SetBayerCvtQuality(1)

            ret = self.camera.MV_CC_StartGrabbing()
            if ret != 0:
                raise RuntimeError(f"MVS start grabbing failed: 0x{ret:08x}")
            self._grabbing = True
        except Exception:
            self.release()
            raise

    def _enable_auto_image_settings(self) -> None:
        """Enable continuous exposure, gain and white-balance adjustment."""
        # GenICam enumeration: Off=0, Once=1, Continuous=2.
        settings = (
            ("ExposureAuto", 2),
            ("GainAuto", 2),
            ("BalanceWhiteAuto", 2),
        )
        for node, value in settings:
            ret = self.camera.MV_CC_SetEnumValue(node, value)
            if ret != 0:
                print(f"⚠ MVS 不支持 {node}=Continuous (0x{ret:08x})")

    def _set_manual_image_settings(self) -> None:
        """Set deterministic exposure and gain for low-light field tests."""
        self.camera.MV_CC_SetEnumValue("ExposureAuto", 0)
        self.camera.MV_CC_SetEnumValue("GainAuto", 0)
        ret = self.camera.MV_CC_SetFloatValue("ExposureTime", self.exposure_us)
        if ret != 0:
            raise RuntimeError(f"MVS set exposure failed: 0x{ret:08x}")
        ret = self.camera.MV_CC_SetFloatValue("Gain", self.gain)
        if ret != 0:
            raise RuntimeError(f"MVS set gain failed: 0x{ret:08x}")

    def isOpened(self) -> bool:
        return self._opened and self._grabbing

    def read(self):
        if not self.isOpened():
            return False, None

        output = MV_FRAME_OUT()
        memset(byref(output), 0, sizeof(output))
        ret = self.camera.MV_CC_GetImageBuffer(output, self.timeout_ms)
        if ret != 0 or not output.pBufAddr:
            return False, None

        try:
            frame_info = output.stFrameInfo
            self.width = frame_info.nWidth
            self.height = frame_info.nHeight
            rgb_size = self.width * self.height * 3
            rgb_buffer = (c_ubyte * rgb_size)()

            params = MV_CC_PIXEL_CONVERT_PARAM_EX()
            memset(byref(params), 0, sizeof(params))
            params.nWidth = self.width
            params.nHeight = self.height
            params.pSrcData = output.pBufAddr
            params.nSrcDataLen = frame_info.nFrameLen
            params.enSrcPixelType = frame_info.enPixelType
            params.enDstPixelType = PixelType_Gvsp_RGB8_Packed
            params.pDstBuffer = rgb_buffer
            params.nDstBufferSize = rgb_size

            ret = self.camera.MV_CC_ConvertPixelTypeEx(params)
            if ret != 0:
                return False, None

            rgb = np.ctypeslib.as_array(rgb_buffer).reshape(self.height, self.width, 3)
            return True, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        finally:
            self.camera.MV_CC_FreeImageBuffer(output)

    def get(self, property_id: int) -> float:
        if property_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.width)
        if property_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.height)
        return 0.0

    def release(self) -> None:
        if self._grabbing:
            self.camera.MV_CC_StopGrabbing()
            self._grabbing = False
        if self._opened:
            self.camera.MV_CC_CloseDevice()
            self._opened = False
        self.camera.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize()
