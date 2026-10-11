# Manual Pusher Failure Recovery

This experimental trigger exercises a standard VTOL's recovery from loss of forward propulsion.
It is a manual substitute for failure detection, not an ESC or battery fault detector.
The flight controller, control surfaces and lift motors must remain operational.
Validate the aircraft's glide and recovery margins in simulation before considering flight testing.

## Setup

Use a standard VTOL (`VT_TYPE=2`) with a separate forward motor.
In the PX4 shell, configure:

```sh
param set SYS_FAILURE_EN 1
param set COM_QC_ACT 1
param set VT_FW_QC_HMAX 50
```

`COM_QC_ACT=1` selects Land after quadchute.
This prototype also uses `VT_FW_QC_HMAX` as its recovery height, captured when the trigger is accepted.
Its existing role as the automatic quadchute height limit is unchanged.
The value of 50 metres is an initial simulation setting, not a validated recovery margin.

Take off, transition to fixed-wing flight, climb above the recovery height, and select Hold.
Choose a clear area and an appropriate `NAV_LOITER_RAD` before triggering.
The existing Hold orbit is retained and flown as a circle.
The existing cruising airspeed is retained; otherwise the normal fixed-wing airspeed target is used.
There is no automatic best-glide speed calculation.

```sh
navigator pusher_failure
listener pusher_failure_status
```

The shell command queues the request.
Navigator reports acceptance or rejection when it processes it.
Acceptance requires armed fixed-wing Hold, a valid position and height reference, no active failsafe, and the configuration above.
Repeated triggers are rejected until disarm.

## Sequence

1. Inhibit the forward motor and command a gliding loiter with airspeed priority.
2. Descend until the recovery height is reached.
3. Request an immediate quadchute, retrying once per second until multicopter mode is observed.
4. Enter Land near the quadchute location using the existing multicopter landing behaviour.

The vehicle does not return to the orbit centre before landing.
Horizontal stopping distance means the touchdown point may differ from the quadchute location.

Height uses valid distance-to-ground when available, otherwise local height above a valid home position.
The `terrain_height` field indicates which reference is being used.
Height above home is not terrain clearance over uneven ground.
If the height reference or navigation position becomes unavailable during the glide, the sequence requests quadchute immediately.
If triggered at or below the recovery height, it also requests quadchute immediately.

The intentional gliding state suppresses only the uncommanded-descent quadchute check while its status is fresh.
Other quadchute protections remain active and can cause earlier recovery.
Other failsafes retain their normal priority.
A mode change away from Hold or Land stops the sequence.
Once Land has been entered, switching back to Hold also stops it.
The pusher remains inhibited until disarm, including after interruption of the sequence.
A new transition from multicopter to fixed-wing mode is prevented while the pusher is inhibited.

`pusher_failure_status` is logged and reports idle, gliding, quadchute requested, landing, aborted, or landed.
It reports commanded recovery state, not proof that an ESC has physically stopped.
The motor stop uses the standard VTOL forward-thrust stop path and leaves lift thrust and control surfaces available.

## Code Locations

- `src/modules/navigator/pusher_failure.cpp`: trigger admission, orbit setpoints, height reference and transition/landing commands.
- `src/modules/navigator/PusherFailureRecovery.hpp`: recovery state machine.
- `src/modules/vtol_att_control/standard.cpp`: forward motor inhibition, latched until disarm.
- `src/modules/vtol_att_control/vtol_type.cpp`: intentional-descent handling.
- `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp`: honours explicit zero-throttle gliding configuration even with a nonzero normal idle-throttle setting.

A future automatic detector should feed the same recovery entry point with a confirmed forward-propulsion fault.
It must distinguish the forward motor from lift motors and arbitrate with Commander's actuator and battery failsafes.
This prototype deliberately does not inject a generic motor failure into those failsafes.
