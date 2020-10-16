
task main()
{
		int x = 2;
		int y = x+100;

		if ( x+y > 103 )
		{
			if ( y >= 100)
			{
				x = 100;
				y = y+x;
			}
			else
				x = y-x;
			y = y-x;
		}
		else
			y = x*8;

		displayString(0, "%d", x);
		displayString(1, "%d", y);

		repeat(forever);
}
