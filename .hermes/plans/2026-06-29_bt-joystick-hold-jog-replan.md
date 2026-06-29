# esp32-step-motor: BT joystick + hold-to-run replan

> **For Hermes:** Follow this plan task-by-task. Keep the current firmware stable, compile after each feature, and do not start the Bluetooth implementation until the transport choice is explicit.

**Goal:** Prepare the next safe iteration of `esp32-step-motor` by first reducing `main/main.c` size, then adding hold-to-run without breaking click behavior or auto-refresh, and only later selecting a Bluetooth control path.

**Architecture:**
The current firmware is already stable for dual motors, Wi-Fi AP/STA, the emergency AP, and OTA. The next change should be split into small verifiable steps. First, move the embedded HTML/JS/CSS out of `main/main.c` into a separate tracked file or generated asset so the main file stops growing. Second, add press-and-hold behavior to the web jog buttons while preserving the existing click action, and make sure the periodic UI refresh never overwrites button interaction state. Third, stop and discuss the Bluetooth transport options before implementing anything, because ESP32-S3 constraints make the choice non-trivial.

**Tech Stack:** ESP-IDF, HTTP server, NVS, FreeRTOS, embedded web UI, Git/GitHub.

---

## Decisions already fixed

- Keep OTA intact.
- Keep Wi-Fi AP/STA and the emergency AP intact.
- Keep the two motors independent.
- Do **not** apply the periodic UI refresh logic to buttons.
- Preserve the current click action on jog buttons; hold must be an additive feature.
- Do not commit to Bluetooth implementation until the transport path is chosen.

## Bluetooth risk discussion to resolve before coding

The ESP32-S3 does **not** support Bluetooth Classic the way a generic phone/gamepad might expect, so the transport matters.

Candidate paths:

1. **BLE HID/gamepad controller**
   - Best fit if the joystick is BLE-compatible.
   - Lowest firmware complexity once the controller is known to work.
   - Likely the cleanest future path for a wireless remote.

2. **PS4 controller**
   - Needs a compatibility check.
   - May be BLE-based, but controller behavior and pairing flow must be validated.
   - Not safe to assume without a small proof-of-concept.

3. **USB joystick into the ESP32-S3**
   - Possible only if the board is used as a USB host and the hardware path supports it.
   - This is riskier on the XIAO ESP32S3 because pins and USB-role constraints matter.
   - Not the first choice unless we explicitly decide to go this route.

4. **External receiver/module**
   - Useful if we want to avoid overusing ESP32-S3 pins later when the display is added.
   - This could offload the remote-control interface, but it adds another board/module.

**Recommendation:**
Pause BLE coding until we choose between BLE HID, PS4 compatibility, USB host, or an external module. For now, only document the trade-offs and test what the board can realistically support.

---

## Task 1: Move the embedded UI out of `main/main.c`

**Objective:** Reduce `main/main.c` size by extracting the HTML/CSS/JS into a separate file or generated asset that can be maintained independently.

**Files:**
- Modify: `main/main.c`
- Create: `main/web_ui.html` or `main/web_ui.inc` or a generated asset file in `main/`
- Modify: `main/CMakeLists.txt` if the build needs to embed the new file

**Implementation notes:**
- Keep the UI exactly functionally equivalent for now.
- Preserve the current page behavior and all endpoints.
- Make sure the build still embeds the page and serves it from `/`.

**Verification:**
- Run `idf.py build`
- Confirm the firmware builds successfully
- Confirm the root page still serves the UI

---

## Task 2: Add hold-to-run to jog buttons without breaking click

**Objective:** Make web jog buttons start motion on press and stop on release while preserving the existing click behavior.

**Files:**
- Modify: the extracted web UI asset from Task 1
- Modify: `main/main.c`

**Implementation notes:**
- Use pointer events if possible (`pointerdown`, `pointerup`, `pointercancel`, `mouseleave`).
- Keep the existing click handler behavior as a fallback so a normal click still works.
- Hold behavior must only be new behavior, not a replacement.
- The stop action must fire on release or cancel.
- Do not let the periodic refresh overwrite pressed-state UI.

**Verification:**
- Build the firmware
- Test that a click still jogs as before
- Test that press-and-hold keeps the motor moving only while pressed
- Test that release stops motion immediately

---

## Task 3: Protect auto-refresh so it does not interfere with buttons

**Objective:** Ensure the UI refresh logic updates status/config fields only and never stomps on button interaction state.

**Files:**
- Modify: the extracted web UI asset from Task 1
- Modify: `main/main.c` only if the state JSON needs extra fields

**Implementation notes:**
- Keep the current field-synchronization pattern for inputs/selects.
- Do not auto-update interactive buttons as if they were input fields.
- Preserve the dirty-field protection already added to editable fields.
- If needed, add a small explicit “pressed” state that is local to the UI and not re-rendered by refresh.

**Verification:**
- Build the firmware
- Confirm periodic refresh still updates status
- Confirm it does not disrupt a held button

---

## Task 4: Decide the Bluetooth path before implementation

**Objective:** Select the remote-control transport with a small proof-of-concept or explicit decision.

**Files:**
- Modify: plan doc only, unless a tiny probe is needed

**Implementation notes:**
- Compare BLE HID/gamepad, PS4 controller behavior, USB host feasibility, and external module options.
- Measure hardware impact: pins, power, USB role, and how much room remains for the future display.
- Do not add full Bluetooth control code yet.

**Verification:**
- Document the selected path
- Record why the other options were rejected

---

## Task 5: Implement Bluetooth only after the transport is chosen

**Objective:** Add the chosen remote-control path with button mapping and hold-to-run semantics.

**Files:**
- TBD after Task 4

**Implementation notes:**
- Support at least 4 mapped buttons.
- Map each button in the UI.
- Keep a watchdog/timeout so a stuck controller cannot leave a motor running forever.
- Preserve OTA, Wi-Fi, and the emergency AP.

**Verification:**
- Build after every Bluetooth subfeature
- Test each step individually before moving on

---

## Execution rule

Do not start the next task until the current task builds cleanly.

## Next recommended order

1. Extract the HTML/UI.
2. Build and verify.
3. Add hold-to-run.
4. Build and verify.
5. Revisit Bluetooth transport choice.
