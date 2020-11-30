Information about SCDTR project group number XX


CanBus communication:
	.messages allow 2 integers of 4 bytes
	.Message id format (11bits)
		id_to1|id_to0|id_from1|id_from0|code7|code6|code5|code4|code3|code2|code1|code0
	.to =00 - all nodes received and responde - general message
	.list of codes:
		(0) 0000000 - Initialization code - all codes must respond - enter configure new node mode
		(1) 0000001 - Respond to prev code
		(2) 0000010 - All at zero and wait for code 3 or code 4
		(3) 0000011 - Turn light at max and wait 5 seconds
		(4) 0000100 - Back to normal mode
		(5) 0000101 - I am the hub
		(6) 0000110 - I am not the hub
		(7) 0000111 - Set to occupied
		(8) 0001000 - Set to unoccupied
		(9) 0001001 - Set to value
		(10) 0001010 - Send me 1 data packet
		(11) 0001011 - Send me a stream of data packets
		(12) 0001100 - Stop the strem

		!!all other codes still available

	codes 0 to 4 are used in wake-up procedure
	codes 5 to 12 in hub function
