MRuby::Gem::Specification.new('esp32-i2c') do |spec|
  spec.license = 'MIT'
  spec.author  = 'Seiei Miyagi'
  spec.summary = 'ESP-IDF I2C library wrapper'
  spec.add_dependency("esp32-ext_esp_err")
end
