from std_msgs.msg import String


class AutoInterventionCommandPublisher:
    """
    Publishes /manual_command commands used by control_node for autonomous-path
    temporary interventions.

    Terminal command behavior:
      a          -> auto toggle: first auto_hold, second auto_active
      a1-a4      -> enter auto_active and execute one auto intervention first
      1/2/3/4    -> auto interventions only when drive_switch_state is auto_*
      0          -> soft stop inside the current mode; in auto mode this means auto_hold
      s/stop     -> hard stop / Stop Mode / disarm

    This class never publishes final /cmd_vel directly.
    """

    AUTO_HOLD = 'auto_hold'
    AUTO_ACTIVE = 'auto_active'
    STOP = 'stop'

    def __init__(self, node, manual_command_pub, drive_mode_pub):
        self.node = node
        self.manual_command_pub = manual_command_pub
        self.drive_mode_pub = drive_mode_pub

    def publish_string(self, publisher, value: str):
        msg = String()
        msg.data = str(value)
        publisher.publish(msg)

    def publish_manual_command(self, command: str):
        self.publish_string(self.manual_command_pub, command)
        self.node.get_logger().info(f'Published /manual_command: {command}')

    def publish_drive_mode(self, mode: str):
        self.publish_string(self.drive_mode_pub, mode)
        self.node.get_logger().info(f'Published /drive_mode: {mode}')

    def handle_auto_toggle(self, current_switch_state: str):
        # Clear any active temporary intervention before changing Auto state.
        self.publish_manual_command('auto')

        if current_switch_state == self.AUTO_HOLD:
            new_state = self.AUTO_ACTIVE
        else:
            new_state = self.AUTO_HOLD

        self.publish_drive_mode(new_state)
        return new_state

    def handle_auto_hold(self):
        # Stop vehicle output while keeping Auto selected.
        # Clear any active control_node intervention.
        self.publish_manual_command('auto')
        self.publish_drive_mode(self.AUTO_HOLD)
        return self.AUTO_HOLD

    def handle_auto_active(self):
        self.publish_manual_command('auto')
        self.publish_drive_mode(self.AUTO_ACTIVE)
        return self.AUTO_ACTIVE

    def handle_intervention(self, command: str, current_switch_state: str):
        """
        Execute a temporary intervention while already in auto mode.
        If the switch is in auto_hold, open it to auto_active first.
        """
        if current_switch_state == self.AUTO_HOLD:
            new_state = self.AUTO_ACTIVE
            self.publish_drive_mode(new_state)
        elif current_switch_state == self.AUTO_ACTIVE:
            new_state = self.AUTO_ACTIVE
            self.publish_drive_mode(new_state)
        else:
            self.node.get_logger().warn(
                f'Received auto intervention "{command}" while switch state is '
                f'"{current_switch_state}". Use "a" first to enter auto mode, '
                f'or use "m" for manual mode.'
            )
            return None

        self.publish_manual_command(command)
        return new_state


    def handle_auto_now(self):
        """
        Rescue/direct auto-active command.
        It opens the auto path immediately without sending a temporary
        1/2/3/4 intervention and asks drive_switch_node to skip the
        transition stop. Use this when you want autonomous control to take
        over right away.
        """
        self.publish_manual_command('auto')
        self.publish_drive_mode('auto_now')
        return self.AUTO_ACTIVE
    def handle_auto_preset(self, command: str):
        """
        a1/a2/a3/a4 shortcut:
          enter auto_active, execute one temporary intervention,
          then control_node returns to autonomous behavior.
        """
        self.publish_manual_command('auto')
        self.publish_drive_mode(self.AUTO_ACTIVE)
        self.publish_manual_command(command)
        return self.AUTO_ACTIVE

    def handle_hard_stop(self):
        # Enter Stop Mode and disarm the drive output.
        self.publish_manual_command('stop')
        self.publish_drive_mode(self.STOP)
        return self.STOP

    # Compatibility alias used by existing terminal code.
    def handle_stop(self):
        return self.handle_hard_stop()
