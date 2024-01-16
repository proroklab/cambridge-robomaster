MAX_VI_RATE=$(cat /sys/kernel/debug/bpmp/debug/clk/vi/max_rate)
MAX_ISP_RATE=$(cat /sys/kernel/debug/bpmp/debug/clk/isp/max_rate)
MAX_NVCSI_RATE=$(cat /sys/kernel/debug/bpmp/debug/clk/nvcsi/max_rate)
MAX_VIC_RATE=$(cat /sys/kernel/debug/bpmp/debug/clk/vic/max_rate)
MAX_EMC_RATE=$(cat /sys/kernel/debug/bpmp/debug/clk/emc/max_rate)

echo $MAX_VI_RATE > /sys/kernel/debug/bpmp/debug/clk/vi/rate
echo $MAX_ISP_RATE > /sys/kernel/debug/bpmp/debug/clk/isp/rate
echo $MAX_NVCSI_RATE > /sys/kernel/debug/bpmp/debug/clk/nvcsi/rate
echo $MAX_VIC_RATE > /sys/kernel/debug/bpmp/debug/clk/vic/rate
echo $MAX_EMC_RATE > /sys/kernel/debug/bpmp/debug/clk/emc/rate

echo 1 > /sys/kernel/debug/bpmp/debug/clk/vi/mrq_rate_locked
echo 1 > /sys/kernel/debug/bpmp/debug/clk/isp/mrq_rate_locked
echo 1 > /sys/kernel/debug/bpmp/debug/clk/nvcsi/mrq_rate_locked
echo 1 > /sys/kernel/debug/bpmp/debug/clk/vic/mrq_rate_locked
echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked   

