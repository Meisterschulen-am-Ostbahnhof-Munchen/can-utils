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

for b in bins:
  p = Program( b, [ b + '.c' ], LIBS=['pthread', 'canutils' ], CFLAGS=[ '-Iinclude' ], LINKFLAGS=['-L.'] )
  Install('/usr/local/bin', b )
  Depends( p, l )


