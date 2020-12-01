Information about SCDTR project group number XX

!!use EEPROM write program to write teh id value before start!!

EEPROM info:
	addr 0 - myID
	addr 1 - number of nodes in the system


CanBus communication:
	.messages allow 2 floats of 4 bytes - define message size of 8
	.Message id format (11bits)
		id_from1|id_from0|code7|code6|code5|code4|code3|code2|code1|code0|id_to1|id_to0
	.to =00 - all nodes received and responde - general message
	.list of codes:
		(0) 0000000 - Initialization code - all codes must respond - enter configure new node mode - all put output at 0
		(1) 0000001 - ack - number of code in the message field
		(2) 0000010 - All get value with message indicates what node has the light turn on
		(3) 0000011 - Turn light at max
		(4) 0000100 - Turn light off
		(5) 0000101 - Back to normal mode
		
		(6) 0000110 - I am the hub
		(7) 0000111 - I am not the hub
		(8) 0001000 - Set to occupied
		(9) 0001001 - Set to unoccupied
		(10) 0001010 - Set to value
		(11) 0001011 - Send me 1 data packet
		(12) 0001100 - Send me a stream of data packets
		(13) 0001101 - Stop the stream
		!!all other codes still available

	codes 0 to 5 are used in wake-up procedure
	codes 6 to 13 in hub function
