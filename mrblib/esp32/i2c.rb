module ESP32
  class I2C
    # execute command
    #
    #     i2c.cmd do |cmd|
    #       cmd.master_start
    #       # ...
    #       cmd.master_stop
    #     end
    def cmd(ticks_to_wait = ::ESP32::I2C::DEFAULT_TICKS_TO_WAIT)
      cmd = ESP32::I2C::CmdHandle.new
      yield cmd
      ret = master_cmd_begin(cmd, ticks_to_wait)
    ensure
      cmd.delete
      ret
    end
  end
end
