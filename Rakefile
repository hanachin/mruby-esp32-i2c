require 'rake/clean'

directory 'mruby' do
  sh 'git clone https://github.com/mruby/mruby.git --depth=1'
end
CLEAN.include('mruby')

task test: %w(mruby)

task default: :test
