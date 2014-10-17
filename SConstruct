# invoke "scons install" to build and install all programs 

bins = [
  'asc2log', 
  'bcmserver', 
  'canbusload', 
  'can-calc-bit-timing', 
  'canconnect', 
  'candump', 
  'canfdtest', 
  'cangen', 
  'cangw', 
  'canlogserver', 
  'canplayer', 
  'cansend', 
  'cansniffer', 
  'isotpdump', 
  'isotprecv', 
  'isotpsend', 
  'isotpserver', 
  'isotpsniffer', 
  'isotptun', 
  'log2asc', 
  'log2long', 
  'slcan_attach', 
  'slcand', 
  'slcanpty' ]

l = Library( 'canutils', [ 'lib.c' ], CFLAGS=[ '-Iinclude' ] )

env = Environment()

for b in bins:
  p = Program( b, [ b + '.c' ], LIBS=['pthread', 'canutils' ], CFLAGS=[ '-Iinclude' ], LINKFLAGS=['-L.'] )
  Depends( p, l )
  env.Install('/usr/local/bin', p )
  env.Alias('install', '/usr/local/bin')


