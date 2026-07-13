0. open serial port

1. send 8x "" to clear buffers (3ms delay)

2. startup sequence:
    all sent messages must end with \r ONLY
    recv messages will not be empty and may end with \r, \n, or \r\n

# Resetting telescope
for the following, assume 8ms delay after send and 105ms delay after recv

    Send '2A01RST'
    Recv
    Send '2A01RST'
    Recv
    Send '2A02RST'
    Recv
    Send '2A03RST'
    Recv

# Enable amplifiers
for the following, assume 8ms delay after send and 5ms delay after recv

    Send '2A01EN1'
    Recv
    Send '2A02EN1'
    Recv
    Send '2A03EN1'
    Recv

# Check software
for the following, assume 8ms delay after send and 5ms delay after recv

    Delay 5ms
    Send 'VER'
    Recv
    Send 'VER'
    Recv str containing 'Ver' indicates Servo controller OK
    Send 'STS'
    Recv str length 20
    Send 'STS'
    Recv str length 20
    Send '2R01'
    Recv nonempty string
    Send '2A01EN'
    Recv str len >= 5 indicates amps OK
    Send '7VDC'
    Recv nonempty string indicates "Feb" OK
    Send 'ERR'
    Recv

3. We should be good to go in terms of startup sequence. In idle state, we can run the following commands in a loop:
8ms delay after send, 0ms delay after recv

    Send 'GAE' // get azimuth and elevation
    Recv 'AZEL, $az, $el'
    Send 'STS' // get long string of status
    Recv
    Send 'ERR' // get az/el error
    Recv 'ERR, $azerr, $elerr'
    Send '2A01SIC'
    Recv current commanded (El)
    Send '2A02SIC'
    Recv current commanded (Az1)
    Send '2A03SIC' 
    Recv current commanded (Az2)
    Send '2A01SIA' 
    Recv current actual (El)
    Send '2A02SIA'
    Recv current actual (Az1)
    Send '2A03SIA' 
    Recv current actual (Az2)
    Send '2A01EN' 
    Recv amp status (El)
    Send '2A02EN' 
    Recv amp status (Az1)
    Send '2A03EN' 
    Recv
    