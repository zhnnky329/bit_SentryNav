class MatchControlStatus:
    def _is_red_side(self):
        return self.spinBox_robot_id.value() < 100

    def get_event_data_value(self):
        """2027 0x0101 第一个 uint32；R/B 是场地位置，不随队色交换。"""
        def rune_status(index):
            return (0, 2, 1)[min(index, 2)]

        red = self._is_red_side()
        value = int(self.checkBox_2.isChecked() or self.auto_supply_zone_detected)
        value |= rune_status(self.comboBox_small_rune.currentIndex()) << 1
        value |= rune_status(self.comboBox_big_rune.currentIndex()) << 3
        # 另一侧可设为 4=未下发，也可在抢占后测试实际增益状态。
        own_fortress = self.comboBox_fortress_buff.currentIndex() & 7
        own_outpost = self.comboBox_outpost_buff.currentIndex() & 7
        other_fortress = self.comboBox_opponent_fortress_gain.currentIndex() & 7
        other_outpost = self.comboBox_opponent_outpost_gain.currentIndex() & 7
        value |= (own_fortress if red else other_fortress) << 5
        value |= (other_fortress if red else own_fortress) << 8
        value |= int(self.checkBox_base_buff.isChecked()) << (11 if red else 14)
        value |= (self.comboBox_central_highland.currentIndex() & 3) << (18 if red else 20)
        value |= (own_outpost if red else other_outpost) << 22
        value |= (other_outpost if red else own_outpost) << 25
        value |= (self.comboBox_r_outpost_control.currentIndex() & 3) << 28
        value |= (self.comboBox_b_outpost_control.currentIndex() & 3) << 30
        return value

    def get_event_data_2_value(self):
        """2027 0x0101 第二个 uint32：基地阵地、飞镖、RMUL 控制区。"""
        value = self.comboBox_r_base_control.currentIndex() & 3
        value |= (self.comboBox_b_base_control.currentIndex() & 3) << 2
        dart_time = min(420, max(0, self._parse_time_to_seconds(
            self.lineEdit_dart_time.get_confirmed_text())))
        value |= (dart_time & 0x1ff) << 4
        target = self.comboBox_dart_target.currentIndex()
        if target == 1 and not self._is_red_side():
            target = 6  # B 前哨站
        value |= (target & 7) << 13
        if self._is_rmul_game_type():
            value |= (self.comboBox_central_buff.currentIndex() & 3) << 26
        return value

    def get_game_status(self):
        self._sync_resurrection_state()
        sentry_info_status = self.get_sentry_info_status()
        status = {
            "game_type": self.comboBox_game_type.currentIndex(),
            "game_progress": self.comboBox_game_stage.currentIndex(),
            "stage_remain_time": self._get_stage_remain_time(),
            "robot_hp": {
                "heavyengineer": self.spinBox_hp_1.value(),
                "reserved_robot": 0,
                "infantry_3": self.spinBox_hp_3.value(),
                "infantry_4": self.spinBox_hp_4.value(),
                "sentry": self.spinBox_hp_7.value(),
                "ally_outpost": self.spinBox_hp_outpost.value(),
                "ally_base": self.spinBox_hp_base.value(),
                "damage_difference": self.spinBox_damage_difference.value(),
                "enemy_outpost": self.spinBox_enemy_outpost_hp.value(),
                "enemy_base": self.spinBox_enemy_base_hp.value(),
                "ally_control_degree": self.spinBox_ally_control_degree.value(),
                "enemy_control_degree": self.spinBox_enemy_control_degree.value(),
            },
            "robot_status": {
                "id": self.spinBox_robot_id.value(),
                "current_hp": self.spinBox_hp_7.value(),
                "max_hp": self.spinBox_hp_7.maximum(),
                "ammo": self.spinBox_ammo.value(),
                "chassis_power_limit": self._current_chassis_power_limit(),
                "shooter_output": self._current_shooter_output(),
            },
            "projectile_allowance": {
                "projectile_allowance_17mm": self.spinBox_ammo.value(),
                "projectile_allowance_42mm": self.spinBox_ammo_42mm.value() if self.spinBox_ammo_42mm else 0,
                "remaining_gold_coin": self.spinBox_gold_coin.value() if self.spinBox_gold_coin else 0,
                "projectile_allowance_fortress": self.spinBox_ammo_fortress.value()
                if self.spinBox_ammo_fortress else 0,
            },
            "event_data": self.get_event_data_value(),
            "event_data_2": self.get_event_data_2_value(),
            "rfid_status": self.get_rfid_status_value(),
            "hurt_data": {
                "armor_id": self.current_hurt_armor_id if self.trigger_hurt else 0,
                "trigger": self.trigger_hurt
            },
            "sentry_can_activate": sentry_info_status["can_activate"],
            "sentry_info": sentry_info_status,
            "power_heat_data": {
                "buffer_energy": self.buffer_energy,
                "shooter_17mm_1_barrel_heat": self.shooter_17mm_heat,
                "shooter_42mm_barrel_heat": self.shooter_42mm_heat,
            },
        }
        if self.trigger_hurt:
            self.trigger_hurt = False

        return status

    def _current_shooter_output(self):
        manual_enabled = (
            self.checkBox_shooter_output.isChecked()
            if hasattr(self, "checkBox_shooter_output")
            else True
        )
        return manual_enabled and not self.sentry_weak_active

    def get_sentry_info_status(self):
        manual_can_activate = (
            self.checkBox_can_activate_rune.isChecked()
            if hasattr(self, "checkBox_can_activate_rune")
            else False
        )
        return {
            "exchange_projectile": self.spinBox_sentry_exchange_projectile.value(),
            "remote_projectile_exchange_count": self.spinBox_sentry_remote_projectile_exchange.value(),
            "remote_hp_exchange_count": self.spinBox_sentry_remote_hp_exchange.value(),
            "can_free_revive": self.checkBox_sentry_can_free_revive.isChecked(),
            "can_buy_revive": self.checkBox_sentry_can_buy_revive.isChecked(),
            "buy_revive_cost": self.spinBox_sentry_buy_revive_cost.value(),
            "is_disengaged": self.checkBox_sentry_is_disengaged.isChecked(),
            "team_projectile_exchange_remaining": self.spinBox_sentry_team_projectile_exchange.value(),
            "can_activate": self.sentry_can_activate_flag or manual_can_activate,
            "current_mode": self.sentry_posture.current_mode,
            "base_posture": (
                self.sentry_posture.current_mode - 3
                if self.sentry_posture.current_mode >= 4
                else self.sentry_posture.current_mode
            ),
            "enhanced": self.sentry_posture.current_mode >= 4,
            "posture_remaining": dict(self.sentry_posture.normal_remaining),
            "enhanced_posture_remaining": dict(self.sentry_posture.enhanced_remaining),
            "sentry_info_3": self.sentry_posture.pack_sentry_info_3(),
        }
