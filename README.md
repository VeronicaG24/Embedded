# Assignment 1 - Embedded Systems

>Due date: 12/11/2023

##### Assignment folder
The assignment is the `UART.X` *MPLAB* project directory.

## Assignment description

- [x] Simulate an algorithm that needs `7ms` for its execution and needs to work at `100Hz`. This is to emulate a real-world scenario. 
- [x] Read characters from `UART2` and display the characters received on the first row of the LCD. 
- [x] When the end of the row has been reached, clear the first row, and start writing again from the first row first column.
- [x] Whenever a CR `\r` or LF `\n` character is received, clear the first row 
- [x] On the second row, write "Char Recv: XXX", where XXX is the number of characters received from the UART2. Use `sprintf(buffer, "%d", value)` to convert an integer to a string to be displayed 
- [x] Whenever button `S5` is pressed, send the current number of chars received to `UART2` 
- [x] Whenever button `S6` is pressed, clear the first row, and reset the characters received counter

### Notes

The following power-up sequence should be observed by the user’s application firmware when writing characters to the LCD: 
1. After any reset operation wait `1000ms` to allow the LCD to begin normal operation. The cursor on the LCD will be positioned at the top row on the left-most column. 
2. Configure `SPI1` module on your dsPIC30F device to operate in 8-bit Master mode. The serial clock may be set for any frequency up to `1MHz`. 
3. To write an ASCII character to the LCD at the location pointed to by the cursor, load the `SPIBUF` register with the ASCII character byte. 
4.	After the character is displayed on the LCD, the cursor is automatically relocated to the next position on the LCD. 
5. To reposition the cursor to another column on any of the two rows, write the address of the desired location to the `SPIBUF` register. Addresses in the first row of the LCD range from `0x80` to `0x8F`, while addresses on the second-row range from `0xC0` through `0xCF`. 
6. After 16 characters are written to the first row on the LCD, it is necessary for the user’s application to write the address `0xC0` of the second row to the `SPIBUF` in order to roll the cursor over to the second row. 
7. The user application must wait for a minimum of (8bits/SPI Frequency) between writing two successive characters or addresses. 

### Considerations

A few things to consider and that are evaluated in the assignment among others:
1. Use of resources, namely memory, and peripherals. How did you size the circular buffer and why?
2. Length of interrupts. Are you doing unnecessary operations, maybe blocking ones?
3. Are you handling shared data with the interrupts correctly?
4. Are you minimizing global variables?
5. Correct use of the UART. Are you managing all the bytes? What if the user sends data continuously?
6. Are you managing the buttons bouncing correctly?
