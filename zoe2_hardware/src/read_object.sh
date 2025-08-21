# SDO read from object dictionary
# make sure you start a candump in another terminal to see the response!
# `candump can0`
# look for a reply with COBID 580 + <CANID>

# sync
cansend can0 80#00.00.00.00.00.00.00.00

# request Motor rated current object data
for CANID in 1 2 3 4
do
	echo "Accessing motor $CANID"
	COBID=$(( $CANID + 600 ))

	# access object 6075, in this example
    cansend can0 $COBID#40.75.60.00.00.00.00.00
done