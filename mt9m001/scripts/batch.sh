for VAR in $(seq 6 30)
do
time sudo ./PRU_memAcc_DDR_sharedRAM 5 -o Ti_12.5_$VAR -d dark12.5average.dat -n 32000
done
