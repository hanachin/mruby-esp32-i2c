require 'rake/clean'

directory 'mruby' do
  sh 'git clone https://github.com/mruby/mruby.git --depth=1'
end
CLEAN.include('mruby')

ESP32_TOOLCHAIN_URL = 'https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz'
directory 'xtensa-esp32-elf' do
  sh "wget -O - #{ESP32_TOOLCHAIN_URL} | tar zxf -"
end

directory 'esp-idf' do
  sh 'git clone --recursive --depth=1 https://github.com/espressif/esp-idf.git'
end

task test: %w(mruby xtensa-esp32-elf esp-idf)

task default: :test
