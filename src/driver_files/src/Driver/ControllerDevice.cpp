#include "ControllerDevice.hpp"
#include <Windows.h>

OculusTouchSteamLink::ControllerDevice::ControllerDevice(std::string serial, Handedness handedness):
    serial_(serial),
    handedness_(handedness)
{
}

std::string OculusTouchSteamLink::ControllerDevice::GetSerial()
{
    return this->serial_;
}

void OculusTouchSteamLink::ControllerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid || !enabled)
        return;

    // Check if this device was asked to be identified
    auto events = GetDriver()->GetOpenVREvents();
    for (auto event : events) {
        // Note here, event.trackedDeviceIndex does not necissarily equal this->device_index_, not sure why, but the component handle will match so we can just use that instead
        //if (event.trackedDeviceIndex == this->device_index_) {
        if (event.eventType == vr::EVREventType::VREvent_Input_HapticVibration) {
            if (event.data.hapticVibration.componentHandle == this->haptic_component_) {
                this->did_vibrate_ = true;
            }
        }
        //}
    }

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count()/1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }

    // Setup pose for this frame
    auto pose = IVRDevice::MakeDefaultPose();

    // Find a HMD
    auto devices = GetDriver()->GetDevices();
    auto hmd = std::find_if(devices.begin(), devices.end(), [](const std::shared_ptr<IVRDevice>& device_ptr) {return device_ptr->GetDeviceType() == DeviceType::HMD; });
    if (hmd != devices.end()) {
        // Found a HMD
        vr::DriverPose_t hmd_pose = (*hmd)->GetPose();

        // Here we setup some transforms so our controllers are offset from the headset by a small amount so we can see them
        linalg::vec<float, 3> hmd_position{ (float)hmd_pose.vecPosition[0], (float)hmd_pose.vecPosition[1], (float)hmd_pose.vecPosition[2] };
        linalg::vec<float, 4> hmd_rotation{ (float)hmd_pose.qRotation.x, (float)hmd_pose.qRotation.y, (float)hmd_pose.qRotation.z, (float)hmd_pose.qRotation.w };

        // Do shaking animation if haptic vibration was requested
        float controller_y = -0.2f + 0.01f * std::sinf(8 * 3.1415f * vibrate_anim_state_);

        // Left hand controller on the left, right hand controller on the right, any other handedness sticks to the middle
        float controller_x = this->handedness_ == Handedness::LEFT ? -0.2f : (this->handedness_ == Handedness::RIGHT ? 0.2f : 0.f);

        linalg::vec<float, 3> hmd_pose_offset = { controller_x, controller_y, -0.5f };

        hmd_pose_offset = linalg::qrot(hmd_rotation, hmd_pose_offset);

        linalg::vec<float, 3> final_pose = hmd_pose_offset + hmd_position;

        pose.vecPosition[0] = final_pose.x;
        pose.vecPosition[1] = final_pose.y;
        pose.vecPosition[2] = final_pose.z;

        pose.qRotation.w = hmd_rotation.w;
        pose.qRotation.x = hmd_rotation.x;
        pose.qRotation.y = hmd_rotation.y;
        pose.qRotation.z = hmd_rotation.z;
    }

    // Check if we need to press any buttons (I am only hooking up the A button here but the process is the same for the others)
    // You will still need to go into the games button bindings and hook up each one (ie. a to left click, b to right click, etc.) for them to work properly
    if (GetAsyncKeyState(0x45 /* E */) != 0) {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, true, 0);
    }
    else {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, false, 0);
    }

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

DeviceType OculusTouchSteamLink::ControllerDevice::GetDeviceType()
{
    return DeviceType::CONTROLLER;
}

OculusTouchSteamLink::Handedness OculusTouchSteamLink::ControllerDevice::GetHandedness()
{
    return this->handedness_;
}

vr::TrackedDeviceIndex_t OculusTouchSteamLink::ControllerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError OculusTouchSteamLink::ControllerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating controller " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "oculus");

    if (this->handedness_ != Handedness::ANY)
    {
        // Setup inputs and outputs
        GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &this->haptic_component_);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/click", &this->a_button_click_component_);
        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/touch", &this->a_button_touch_component_);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/click", &this->b_button_click_component_);
        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/touch", &this->b_button_touch_component_);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/x/click", &this->x_button_click_component_);
        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/x/touch", &this->x_button_touch_component_);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/y/click", &this->y_button_click_component_);
        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/y/touch", &this->x_button_touch_component_);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/click", &this->trigger_click_component_);
        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/touch", &this->trigger_touch_component_);
        GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trigger/value", &this->trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

        GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/value", &this->grip_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &this->system_click_component_);

        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/click", &this->joystick_click_component_);
        GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/touch", &this->joystick_touch_component_);
        GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/x", &this->joystick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
        GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/y", &this->joystick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    }

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);

    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "oculus_touch");

    const ovrHmdType oculusHMDType = ovr_GetHmdDesc(GetDriver()->oculusVRSession).Type;
    std::string oculusHMDString;
    switch (oculusHMDType)
    {
    case ovrHmdType::ovrHmd_CV1:
        oculusHMDString = "cv1";
        break;
    case ovrHmdType::ovrHmd_Quest:
        oculusHMDString = "quest";
        break;
    case ovrHmdType::ovrHmd_Quest2:
        oculusHMDString = "quest2";
        break;
    case ovrHmdType::ovrHmd_RiftS:
        oculusHMDString = "rifts";
        break;
    default:
        oculusHMDString = "cv1"; //Default to CV1 resources.
        break;
    }

    // Give SteamVR a hint at what hand this controller is for
    if (this->handedness_ == Handedness::LEFT)
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, ("oculus_" + oculusHMDString + "_controller_left").c_str());
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_LeftHand);
    }
    else if (this->handedness_ == Handedness::RIGHT)
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, ("oculus_" + oculusHMDString + "_controller_right").c_str());
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_RightHand);
    }
    else
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{oculus_touch_steam_link}/rendermodels/example_controller");
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);
    }

    // Set controller profile
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");

    //Try to get OpenVR (SteamVR) to prefer other controllers over these ones (incase the user wants to use this plugin in tracker mode).
    //vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, 0);

    //Icons
    std::string handString = this->handedness_ == Handedness::LEFT ? "left" : "right";
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_ready.png").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_off.png").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_searching.gif").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_searching_alert.gif").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_ready_alert.png").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_error.png").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_off.png").c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, ("{oculus}/icons/" + oculusHMDString + "_" + handString + "_controller_ready_low.png").c_str());

    return vr::EVRInitError::VRInitError_None;
}

void OculusTouchSteamLink::ControllerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void OculusTouchSteamLink::ControllerDevice::EnterStandby()
{
}

void* OculusTouchSteamLink::ControllerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void OculusTouchSteamLink::ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t OculusTouchSteamLink::ControllerDevice::GetPose()
{
    return last_pose_;
}
