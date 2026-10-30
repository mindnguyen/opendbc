"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen import pqcan, mqbcan, mebcan
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else (mebcan if CP.flags & VolkswagenFlags.MEB else mqbcan)
    # MQB Evo steering-wheel buttons have dedicated +/- tips; RES/SET would not step the set speed (logged 2026-10-30)
    self.tip_buttons = bool(CP.flags & VolkswagenFlags.MQB_EVO)  # the CarController later swaps self.CCS to mebcan for MQB Evo
    self.press_start_frame = -1000

  @staticmethod
  def tip_message(packer, bus, gra_stock_values, up, down):
    """GRA_ACC_01 with the dedicated +/- tips (steering-wheel buttons).  Self-contained on purpose: it must not
    depend on the signature of the CAN helper module, a mismatch there killed card on 2026-10-30."""
    values = {s: gra_stock_values[s] for s in ("GRA_Hauptschalter", "GRA_Typ_Hauptschalter", "GRA_Codierung",
                                               "GRA_Tip_Stufe_2", "GRA_ButtonTypeInfo")}
    values.update({"COUNTER": (gra_stock_values["COUNTER"] + 1) % 16, "GRA_Abbrechen": False, "GRA_Tip_Wiederaufnahme": False,
                   "GRA_Tip_Setzen": False, "GRA_Tip_Hoch": bool(up), "GRA_Tip_Runter": bool(down)})
    return packer.make_can_msg("GRA_ACC_01", bus, values)

  def update(self, CC_SP, CS, packer, frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    up = self.ICBM.sendButton == SendButtonState.increase
    down = self.ICBM.sendButton == SendButtonState.decrease

    # make sure cruise state is already enabled to not enable car cruise user unintended
    if not (CS.out.cruiseState.enabled and (up or down)):
      return can_sends

    # NOTE: the VW CarController calls this method with itself as `self`; keep all state lazy and never raise:
    # an exception here takes card (and with it steering) down.
    try:
      tip_buttons = getattr(self, "tip_buttons", None)
      if tip_buttons is None:
        # MQB Evo steering-wheel buttons: RES/SET do not step the set speed, the +/- tips do (logged 2026-10-30)
        tip_buttons = bool(self.CP.flags & VolkswagenFlags.MQB_EVO)
        self.tip_buttons = tip_buttons
      if tip_buttons:
        # hold the tip for ~90 ms (about 3 stock frames) so the car registers it, one tap every 250 ms
        start = getattr(self, "press_start_frame", -1000)
        t = (frame - start) * DT_CTRL
        if t > 0.25:
          self.press_start_frame = frame
          t = 0.0
        if t < 0.09:
          can_sends.append(IntelligentCruiseButtonManagementInterface.tip_message(packer, CAN, CS.gra_stock_values, up, down))
          self.last_button_frame = frame
      elif (frame - self.last_button_frame) * DT_CTRL > 0.2:
        # stalk cars: set and resume achieve -1 and +1
        can_sends.append(self.CCS.create_acc_buttons_control(packer, CAN, CS.gra_stock_values, up=up, down=down))
        self.last_button_frame = frame
    except Exception as e:
      n = getattr(self, "_icbm_errors", 0) + 1
      self._icbm_errors = n
      if n <= 3:
        print(f"ICBM: button send failed, skipping ({e!r})", flush=True)
      return []
    return can_sends
