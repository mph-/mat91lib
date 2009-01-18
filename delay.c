void delay_us (unsigned int us)
{


}



void delay_ms (unsigned int ms)
{
    while (ms)
    {
        DELAY_US (1000);
        ms--;
    }
}
