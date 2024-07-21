********************************************************************
* Demonstration Program *                                          *
* Authors: Krish Patel, Simrat Gill, Sanjay Sivapragasm *          *
* eeBOT 31445                                                      *
* Tuesday November 28, 2023 10:14am                                *
* Section #4 - TA: Mohsen Ensafjoo                                 *
********************************************************************
; export symbols
                    XDEF Entry, _Startup    ; export 'Entry' symbol
                    ABSENTRY Entry          ; for absolute assembly: mark this as application entry point
                    INCLUDE "derivative.inc"

; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME          EQU $01                 ; Clear the display and home the cursor
INTERFACE           EQU $38                 ; 8 bit interface, two line display
CURSOR_OFF          EQU $0C                 ; Display on, cursor off
SHIFT_OFF           EQU $06                 ; Address increments, no character shift
LCD_SEC_LINE        EQU 64                  ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
LCD_CNTR            EQU PTJ                 ; LCD Control Register: E = PJ7, RS = PJ6 
LCD_DAT             EQU PORTB               ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E               EQU $80                 ; LCD E-signal pin
LCD_RS              EQU $40                 ; LCD RS-signal pin

; Other codes
NULL                EQU 00                  ; The string 'null terminator'
CR                  EQU $0D                 ; 'Carriage Return' character
SPACE               EQU ' '                 ; The 'space' character

; Timers
T_LEFT              EQU 8
T_RIGHT             EQU 8

; States
START               EQU 0
FWD                 EQU 1
ALL_STP             EQU 2
L_TRN               EQU 3
R_TRN               EQU 4
REV                 EQU 5
L_ALGN              EQU 6
R_ALGN              EQU 7

;********************************************************************************************************
; Variable Section
;********************************************************************************************************
                    ORG $3800
           
; Initial values
BASE_LINE           FCB $9D                 ;EF value occurs when on white
BASE_BOW            FCB $CA                 ;A
BASE_MID            FCB $CA                 ;C
BASE_PORT           FCB $CC                 ;B
BASE_STBD           FCB $CC                 ;D

LINE_VARIANCE       FCB $18                 ; difference between black vs white
BOW_VARIANCE        FCB $30                 
PORT_VARIANCE       FCB $20
MID_VARIANCE        FCB $20
STARBOARD_VARIANCE  FCB $15

; Display settings
TOP_LINE        RMB     20                  ;Top Line on the Display
                FCB     NULL                ;Terminated by 'null'

BOT_LINE        RMB     20                  ;Bottom Line on the Display
                FCB     NULL                ;Terminated by 'null'

CLEAR_LINE      FCC     ''                  ;Clear the Display
                FCB     NULL                ;Terminated by 'null'

TEMP            RMB     1                   ;Temp Location

; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)
SENSOR_LINE     FCB     $01                 ; Storage for guider sensor readings
SENSOR_BOW      FCB     $23                 ; Initialized to test values
SENSOR_PORT     FCB     $45
SENSOR_MID      FCB     $67
SENSOR_STBD     FCB     $89
SENSOR_NUM      RMB     1                   ; The currently selected sensor

; List of Variables
                ORG     $3850              ; Where our TOF counter register lives
TOF_COUNTER     dc.b    0                  ; The timer, incremented at 23Hz
CRNT_STATE      dc.b    2                  ; Current state register
T_TURN          ds.b    1                  ; FWD time
TEN_THOUS       ds.b    1                  ; 10,000 digit
THOUSANDS       ds.b    1                  ; 1,000 digit
HUNDREDS        ds.b    1                  ; 100 digit
TENS            ds.b    1                  ; 10 digit
UNITS           ds.b    1                  ; 1 digit
NO_BLANK        ds.b    1                  ; Used in 'leading zero' blanking by BCD2ASC
BCD_SPARE       RMB     2                  ; Extra space for decimal point and string terminator
HEX_TABLE       FCC     '0123456789ABCDEF'  ;Table for converting values to HEX
                          
;********************************************************************************************************
; Code Section
;********************************************************************************************************
                ORG     $4000
;               Initialization
Entry:
_Startup:
                LDS     #$4000              ; Initialize the stack pointer
                CLI                         ; Enable interrupts 
                JSR     INIT                ; Initialize ports
                JSR     openADC             ; Initialize the ATD
                JSR     initLCD             ; Initialize the LCD
                JSR     CLR_LCD_BUF         ; Write 'space' characters to the LCD buffer
                BSET    DDRA,%00000011      ; STAR_DIR, PORT_DIR               
                BSET    DDRT,%00110000      ; STAR_SPEED, PORT_SPEED           
                JSR     initAD              ; Initialize ATD converter                                        
                JSR     initLCD             ; Initialize the LCD                       
                JSR     clrLCD              ; Clear LCD & home cursor                                                   
                LDX     #msg1               ; Display msg1                               
                JSR     putsLCD             ; "                                               
                LDAA    #$C0                ; Move LCD cursor to the 2nd row             
                JSR     cmd2LCD                                           
                LDX     #msg2               ; Display msg2                               
                JSR     putsLCD             ; "                                        
                JSR     ENABLE_TOF          ; Jump to TOF initialization
                
;               Display Sensors                
MAIN
                JSR     G_LEDS_ON           ; Enable the guider LEDs
                JSR     READ_SENSORS        ; Read the 5 guider sensors
                JSR     G_LEDS_OFF          ; Disable the guider LEDs
                JSR     UPDT_DISPL           ; and write them to the LCD
                LDAA    CRNT_STATE         
                JSR     DISPATCHER 
                BRA     MAIN                ; Loop forever

; data section
msg1            dc.b    "Battery volt ",0
msg2            dc.b    "State ",0
tab             dc.b    "START  ",0
                dc.b    "FWD    ",0
                dc.b    "ALL_STP    ",0
                dc.b    "L_TRN",0
                dc.b    "R_TRN",0
                dc.b    "REV",0
                dc.b    "L_TIME", 0
                dc.b    "R_TIME", 0 
                


;********************************************************************************************************
; Sub-Routine Section
;********************************************************************************************************

DISPATCHER  	JSR	  	VERIFY_START
                RTS
    
VERIFY_START   	CMPA    #START            ;Verify START
                BNE     VERIFY_FWD        ; Branch to FWD if not 
                JSR     START_ST          ; Confirm START
                RTS

VERIFY_FWD     	CMPA    #FWD              ; Same concept as VERIFY_START
            	BNE     VERIFY_STP
           		JSR     FWD_ST
              	RTS

VERIFY_REV     	CMPA    #REV              ; Same concept as VERIFY_START
              	BNE     VERIFY_L_ALGN
              	JSR     REV_ST
              	RTS

VERIFY_STP 	  	CMPA    #ALL_STP          ; Same concept as VERIFY_START
              	BNE     VERIFY_L_TRN
              	JSR     ALL_STOP_ST
            	RTS                                   

VERIFY_L_TRN 	CMPA    #L_TRN            ; Same concept as VERIFY_START
                BNE     VERIFY_R_TRN
                JSR     LEFT           
                RTS                
            
VERIFY_L_ALGN 	CMPA    #L_ALGN           ; Same concept as VERIFY_START
                BNE     VERIFY_R_ALGN
                JSR     L_ALGN_DONE     
                RTS                
            
VERIFY_R_TRN 	CMPA    #R_TRN            ; Same concept as VERIFY_START
                BNE     VERIFY_REV
                JSR     RIGHT           
            
VERIFY_R_ALGN 	CMPA    #R_ALGN           ; Same concept as VERIFY_START
                JSR     R_ALGN_DONE       ; Invalid State
                RTS
;********************************************************************************************************
; Movement
;********************************************************************************************************  
START_ST	    	BRCLR   PORTAD0, %00000100, RELEASE
				        JSR   	INIT_FWD
                MOVB	  #FWD, CRNT_STATE
                
RELEASE		    	RTS
;********************************************************************************************************                  

FWD_ST		    BRSET 	    PORTAD0, $04, NO_FWD_BUMP 	    ; Checks if bow bumper is hit
                MOVB	    #REV, CRNT_STATE	            ; if true, enter the REV_TRN state
                
                JSR		    UPDT_DISPL					    ; update the display
                JSR		    INIT_REV
                LDY	  	    #6000
                JSR	  	    del_50us
                JSR	  	    INIT_RIGHT
                LDY		    #6000
                JSR		    del_50us
                JMP         EXIT
                
NO_FWD_BUMP     BRSET 	PORTAD0, $04, NO_FWD_REAR_BUMP	; checks if stern bumper is hit
                MOVB  	#ALL_STP, CRNT_STATE			; if true enter ALL_STOP state
                JSR	  	INIT_STOP
                JMP	    EXIT

NO_FWD_REAR_BUMP                                  ; Check sensors and commit adjustments
                LDAA  	SENSOR_BOW                ; to keep bot centered as much as possible
                ADDA  	BOW_VARIANCE              
                CMPA  	BASE_BOW
                BPL	  	NOT_ALGN
                LDAA	  SENSOR_MID
                ADDA	  MID_VARIANCE
                CMPA  	BASE_MID
                BPL		  NOT_ALGN
                LDAA  	SENSOR_LINE
                ADDA	  LINE_VARIANCE
                CMPA  	BASE_LINE
                BPL	  	CHECK_R_ALGN
                LDAA  	SENSOR_LINE
                SUBA  	LINE_VARIANCE
                CMPA  	BASE_LINE
                BMI	  	CHECK_L_ALGN
                
;********************************************************************************************************

NOT_ALGN  	  	LDAA  	SENSOR_PORT             ; Check sensors and commit adjustments
				        ADDA  	PORT_VARIANCE
                CMPA  	BASE_PORT
                BPL	  	PARTIAL_L_TRN
                BMI     NO_PORT
NO_PORT         LDAA    SENSOR_BOW
                ADDA    BOW_VARIANCE
                CMPA    BASE_BOW
                BPL     EXIT
                BMI     NO_BOW
              
NO_BOW          LDAA    SENSOR_STBD
                ADDA    STARBOARD_VARIANCE
                CMPA    BASE_STBD
                BPL     PARTIAL_R_TRN
                BMI     EXIT

;********************************************************************************************************
PARTIAL_L_TRN   LDY     #6000                   ; Adjust partially left and check alignment
                JSR     del_50us
                JSR     INIT_LEFT
                MOVB    #L_TRN, CRNT_STATE
                LDY     #6000
                JSR     del_50us
                BRA     EXIT

CHECK_L_ALGN    JSR     INIT_LEFT
                MOVB    #L_ALGN, CRNT_STATE
                BRA     EXIT

;********************************************************************************************************
PARTIAL_R_TRN   LDY     #6000                   ; Adjust partially right and check alignment
                JSR     del_50us
                JSR     INIT_RIGHT
                MOVB    #R_TRN, CRNT_STATE
                LDY     #6000
                JSR     del_50us
                BRA     EXIT

CHECK_R_ALGN    JSR     INIT_RIGHT
                MOVB    #R_TRN, CRNT_STATE
                BRA     EXIT
                
EXIT            RTS

;********************************************************************************************************
LEFT            LDAA    SENSOR_BOW              ; Turn Left and check alignment
                ADDA    BOW_VARIANCE
                CMPA    BASE_BOW
                BPL     L_ALGN_DONE
                BMI     EXIT

L_ALGN_DONE     MOVB    #FWD, CRNT_STATE        
                JSR     INIT_FWD
                BRA     EXIT

RIGHT           LDAA    SENSOR_BOW              ; Turn Right and check alignment
                ADDA    BOW_VARIANCE
                CMPA    BASE_BOW
                BPL     R_ALGN_DONE
                BMI     EXIT
                        
R_ALGN_DONE     MOVB    #FWD, CRNT_STATE
                JSR     INIT_FWD
                BRA     EXIT

;********************************************************************************************************

REV_ST          LDAA    SENSOR_BOW              ; Reverse, turn, and go forward
                ADDA    BOW_VARIANCE
                CMPA    BASE_BOW
                BMI     EXIT
                JSR     INIT_LEFT
                MOVB    #FWD, CRNT_STATE
                JSR     INIT_FWD
                BRA     EXIT

ALL_STOP_ST     BRSET   PORTAD0, $04, NO_START_BUMP
                MOVB    #START, CRNT_STATE

NO_START_BUMP   RTS

;********************************************************************************************************
INIT_RIGHT      BSET    PORTA, %00000010         ; Initialize motors
                BCLR    PORTA, %00000001
                LDAA    TOF_COUNTER
                ADDA    #T_RIGHT
                STAA    T_TURN
                RTS

INIT_LEFT       BSET    PORTA, %00000001
                BCLR    PORTA, %00000010
                LDAA    TOF_COUNTER
                ADDA    #T_LEFT
                STAA    T_TURN
                RTS

INIT_FWD        BCLR    PORTA, %00000011
                BSET    PTT, %00110000
                RTS
                
INIT_REV        BCLR    PORTA, %00000011
                BSET    PTT, %00110000
                RTS
                
INIT_STOP       BCLR    PTT, %00110000
                RTS
                
;********************************************************************************************************
; initialize sensors
INIT            BCLR  DDRAD, $FF                ; Make PORTAD an input (DDRAD @ $0272)
                BSET  DDRA, $FF                 ; Make PORTA an output (DDRA @)$0002

                BSET  DDRB, $FF                 ; Make PORTB an output (DDRB @ $0003)
                BSET  DDRA, $C0                 ; Make pins 7, 6 of PTJ outputs (DDRJ @ $026A)
                RTS
        
;********************************************************************************************************
; initialize ADC
openADC         MOVB    #$80, ATDCTL2   ; Turn on ADC 
                LDY     #1              ; Wait 50 us         
                JSR     del_50us
                MOVB    #$20, ATDCTL3   ; 4 conversions on channel AN1
                MOVB    #$97, ATDCTL4   ; 8-bit resolution
                RTS
;********************************************************************************************************
;Clear LCD Buffer
CLR_LCD_BUF     LDX     #CLEAR_LINE
                LDY     #TOP_LINE
                JSR     STRCPY

CLB_SECOND      LDX     #CLEAR_LINE
                LDY     #BOT_LINE
                JSR     STRCPY

CLB_EXIT        RTS          
;********************************************************************************************************

; String Copy
STRCPY          PSHX
                PSHY
                PSHA

STRCPY_LOOP     LDAA    0,X
                STAA    0,Y
                BEQ     STRCPY_EXIT
                INX
                INY
                BRA     STRCPY_LOOP

STRCPY_EXIT     PULA
                PULY
                PULX 
                RTS
;********************************************************************************************************
;                 Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor correspond to the ?illuminated?
; situation. Passed: Nothing. Returns: Nothing.
; Side: PORTA bit 5 is changed
G_LEDS_ON       BSET    PORTA, %00100000  ; set bit 5
                RTS


;********************************************************************************************************

;                 Guider LEDs OFF
; This routine disables the guider LEDs. Readings of the sensor correspond to the ?ambient lighting?
; situation. Passed: Nothing. Returns: Nothing.
; Side: PORTA bit 5 is changed
G_LEDS_OFF      BCLR    PORTA, %00100000  ; clear bit 5
                RTS

;********************************************************************************************************
;                Read Sensors

READ_SENSORS    CLR     SENSOR_NUM        ; select sensor number 0
                LDX     #SENSOR_LINE      ; Point at the start of the sensor array

RS_MAIN_LOOP    LDAA    SENSOR_NUM        ; select the correct sensor input
                JSR     SELECT_SENSOR     ; on the hardware
                LDY     #400              ; 20 ms delay to allow the
                JSR     del_50us          ; sensor to stablize
                LDAA    #%10000001        ; Start A/D conversion on AN1
                STAA    ATDCTL5
                BRCLR   ATDSTAT0, $80, *  ; Repeat until A/D signals done
                LDAA    ATDDR0L            ; A/D conversion is complete in ATDDROL
                STAA    0, X              ; so copy it to the sensor register
                CPX     #SENSOR_STBD      ; if this is the last reading then exit
                BEQ     RS_EXIT
                INC     SENSOR_NUM        ; else, increment the sensor number
                INX                       ; and the pointer into the sensor array
                BRA     RS_MAIN_LOOP      ; and do it again

RS_EXIT         RTS

;********************************************************************************************************
;                               Select Sensor
SELECT_SENSOR   PSHA                    ; save the sensor number for the moment
                LDAA    PORTA           ; clear the sensor selecion bits to zeros
                ANDA    #%11100011
                STAA    TEMP            ; and save it into TEMP
                PULA                    ; get the sensor number
                ASLA                    ; shift the selection number left, twice
                ASLA                    
                ANDA    #%00011100      ; clear irrelevant bit positions
                ORAA    TEMP            ; OR it into the sensor bit positions
                STAA    PORTA           ; Update the hardware
                RTS  

;********************************************************************************************************
;                               Display Sensor

DP_FRONT_SENSOR EQU     TOP_LINE+3
DP_PORT_SENSOR  EQU     TOP_LINE+0
DP_MID_SENSOR   EQU     TOP_LINE+3
DP_STBD_SENSOR  EQU     TOP_LINE+6
DP_LINE_SENSOR  EQU     TOP_LINE+9    

DISPLAY_SENSORS         
                LDAA    SENSOR_BOW      ; get the front sensor value
                JSR     BIN2ASC         ; convert to ascii string in D
                LDX     #DP_FRONT_SENSOR; point to LCD buffer position
                STD     0, X            ; and write the 2 ascii digits there
                LDAA    SENSOR_PORT     ; repeat for the port value
                JSR     BIN2ASC
                LDX     #DP_PORT_SENSOR
                STD     0, X
                LDAA    SENSOR_MID      ; repeat for MID value
                JSR     BIN2ASC
                LDX     #DP_MID_SENSOR
                STD     0, X
                LDAA    SENSOR_STBD     ; repeat for starboard value
                JSR     BIN2ASC
                LDX     #DP_STBD_SENSOR
                STD     0, X
                LDAA    SENSOR_LINE     ; repeat for LINE value
                JSR     BIN2ASC
                LDX     #DP_LINE_SENSOR
                STD     0, X
                LDAA    #CLEAR_HOME     ;Clear the display and home the cursor
                JSR     cmd2LCD         
                LDY     #40             ; wait 2 ms until clear display command is complete
                JSR     del_50us        
                LDX     #TOP_LINE       ; Now copy the buffer top line to the LCD
                JSR     putsLCD
                LDAA    #LCD_SEC_LINE   ; position LCD cursor on 2nd line
                JSR     LCD_POS_CRSR
                LDX     #BOT_LINE       ; copy the buffer bottom line to the LCD
                JSR     putsLCD
                RTS
;********************************************************************************************************
;* Update Display (Battery Voltage + Current State) *
;********************************************************************************************************
UPDT_DISPL              
                MOVB    #$90,ATDCTL5    ; R-just., uns., sing. conv., mult., ch=0, start
                BRCLR   ATDSTAT0,$80,*  ; Wait until the conver. seq. is complete
                LDAA    ATDDR0L         ; Load the ch0 result - battery volt - into A
                LDAB    #39             ;AccB = 39
                MUL                     ;AccD = 1st result x 39
                ADDD    #600            ;AccD = 1st result x 39 + 600
                JSR     int2BCD
                JSR     BCD2ASC
                LDAA    #$8D            ;move LCD cursor to the 1st row, end of msg1
                JSR     cmd2LCD         ;"
                LDAA    TEN_THOUS       ;output the TEN_THOUS ASCII character
                JSR     putcLCD         ;"
                LDAA    THOUSANDS
                JSR     putcLCD
                LDAA    #'.'
                JSR     putcLCD
                LDAA    HUNDREDS
                JSR     putcLCD         ; Display the battery voltage
                ;-------------------------
                LDAA    #$C7            ; Move LCD cursor to the 2nd row, end of msg2
                JSR     cmd2LCD 
                LDAB    CRNT_STATE      ; Display current state
                LSLB 
                LSLB 
                LSLB 
                LDX     #tab 
                ABX 
                JSR     putsLCD
                RTS            
                                                
;********************************************************************************************************
ENABLE_TOF      LDAA    #%10000000
                STAA    TSCR1           ; Enable TCNT
                STAA    TFLG2           ; Clear TOF
                LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                STAA    TSCR2
                RTS

;********************************************************************************************************
TOF_ISR         INC     TOF_COUNTER
                LDAA    #%10000000      ; Clear
                STAA    TFLG2           ; TOF
                RTI

; utility subroutines
;*******************************************************************
;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display, *
;* turn on display, cursor and blinking off. Shift cursor right. *
;*******************************************************************
initLCD     
                BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                LDY     #2000           ; wait for LCD to be ready
                JSR     del_50us        ; -"-
                LDAA    #$38            ; set 8-bit data, 2-line display
                JSR     cmd2LCD         ; -"-
                LDAA    #$0C            ; display on, cursor off, blinking off
                JSR     cmd2LCD         ; -"-
                LDAA    #$06            ; move cursor right after entering a character
                JSR     cmd2LCD         ; -"-
                RTS

;*******************************************************************
;* Clear display and home cursor *
;*******************************************************************
clrLCD          LDAA    #$01            ; clear cursor and return to home position
                JSR     cmd2LCD         ; -"-
                LDY     #40             ; wait until "clear cursor" command is complete
                JSR     del_50us        ; -"-
                RTS

;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns. *
;*******************************************************************
del_50us:       PSHX                    ;2 E-clk
eloop:          LDX #300                ;2 E-clk -
iloop:          NOP
                DBNE X,iloop            ;3 E-clk -
                DBNE Y,eloop            ;3 E-clk
                PULX                    ;3 E-clk
                RTS                     ;5 E-clk

;*******************************************************************
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:        BCLR LCD_CNTR,LCD_RS    ; select the LCD Instruction Register (IR)
                JSR dataMov             ; send data to IR
                RTS
           
;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X *
;*******************************************************************
putsLCD         LDAA 1,X+               ; get one character from the string
                BEQ donePS              ; reach NULL character?
                JSR putcLCD
                BRA putsLCD
donePS          RTS


;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *
;*******************************************************************
putcLCD         BSET LCD_CNTR,LCD_RS    ; select the LCD Data register (DR)
                JSR dataMov             ; send data to DR
                RTS
           
;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov         BSET LCD_CNTR,LCD_E     ; pull the LCD E-sigal high
                STAA LCD_DAT            ; send the upper 4 bits of data to LCD
                BCLR LCD_CNTR,LCD_E     ; pull the LCD E-signal low to complete the write oper.
                LSLA                    ; match the lower 4 bits with the LCD data pins
                LSLA 
                LSLA 
                LSLA 
                BSET LCD_CNTR,LCD_E     ; pull the LCD E signal high
                STAA LCD_DAT            ; send the lower 4 bits of data to LCD
                BCLR LCD_CNTR,LCD_E     ; pull the LCD E-signal low to complete the write oper.
                LDY #1                  ; adding this delay will complete the internal
                JSR del_50us            ; operation for most instructions
                RTS

initAD          MOVB #$C0,ATDCTL2       ;power up AD, select fast flag clear
                JSR del_50us            ;wait for 50 us
                MOVB #$00,ATDCTL3       ;8 conversions in a sequence
                MOVB #$85,ATDCTL4       ;res=8, conv-clks=2, prescal=12
                BSET ATDDIEN,$0C        ;configure pins AN03,AN02 as digital inputs
                RTS

;*****************************************************************
int2BCD         XGDX                    ; Save the binary number into .X
                LDAA #0                 ; clear the BCD_BUFFER
                STAA TEN_THOUS
                STAA THOUSANDS
                STAA HUNDREDS
                STAA TENS
                STAA UNITS
                STAA BCD_SPARE
                STAA BCD_SPARE+1
                
                CPX #0                  ; Check for a zero input
                BEQ CON_EXIT            ; and if so, exit

                XGDX                    ; Not zero, get the binary number back to .D as dividend
                LDX #10                 ; Setup 10 (Decimal!) as the divisor
                IDIV                    ; Divide: Quotient is now in .X, remainder in .D
                STAB UNITS              ; Store remainder
                CPX #0                  ; If quotient is zero,
                BEQ CON_EXIT            ; then exit

                XGDX                    ; swap first quotient back into .D
                LDX #10                 ; and setup for another divide by 10
                IDIV
                STAB TENS
                CPX #0
                BEQ CON_EXIT

                XGDX                    ; Swap quotient back into .D
                LDX #10                 ; and setup for another divide by 10
                
                IDIV
                STAB HUNDREDS
                CPX #0
                BEQ CON_EXIT

                XGDX                    ; Swap quotient back into .D
                LDX #10                 ; and setup for another divide by 10
                IDIV
                STAB THOUSANDS
                CPX #0
                BEQ CON_EXIT

                XGDX                    ;Swap quotient back into .D
                LDX #10                 ;and setup for another divide by 10
                IDIV
                STAB TEN_THOUS

CON_EXIT        RTS                     ; Were done the conversion

LCD_POS_CRSR    ORAA #%10000000         ; Set the high bit of the control word
                JSR  cmd2LCD            ;  and set the cursor address    
                RTS

BIN2ASC         PSHA                    ; Save a copy of the input number on the stack
                TAB                     ;  and copy it into ACCB
                ANDB #%00001111         ; Strip off the upper nibble of ACCB
                CLRA                    ; D now contains 000n where n is the LSnibble
                ADDD #HEX_TABLE         ; Set up for indexed load
                XGDX                
                LDAA 0,X                ; Get the LSnibble character

                PULB                    ; Retrieve the input number into ACCB
                PSHA                    ; and push the LSnibble character in its place
                RORB                    ; Move the upper nibble of the input number
                RORB                    ;  into the lower nibble position.
                RORB
                RORB 
                ANDB #%00001111         ; Strip off the upper nibble
                CLRA                    ; D now contains 000n where n is the MSnibble 
                ADDD #HEX_TABLE         ; Set up for indexed load
                XGDX                                                                
                LDAA 0,X                ; Get the MSnibble character into ACCA
                PULB                    ; Retrieve the LSnibble character into ACCB
                RTS

openLCD         LDY  #2000              ; Wait 100 ms for LCD to be ready
                JSR  del_50us           ;       "
                LDAA #INTERFACE         ; Set 8-bit data, 2-line display, 5x8 font
                JSR  cmd2LCD            ;       "
                LDAA #CURSOR_OFF        ; Display on, cursor off, blinking off
                JSR  cmd2LCD            ;       "
                LDAA #SHIFT_OFF         ; Move cursor right (address increments, no char. shift)
                JSR  cmd2LCD            ;       "
                LDAA #CLEAR_HOME        ; Clear the display and home the cursor
                JSR  cmd2LCD            ;       "
                LDY  #40                ; Wait 2 ms until "clear display" command is complete
                JSR  del_50us           ;       "
                RTS 

BCD2ASC         LDAA  #0                ; Initialize the blanking flag
                STAA NO_BLANK

C_TTHOU         LDAA TEN_THOUS          ;Check the ten_thousands digit
                ORAA NO_BLANK
                BNE NOT_BLANK1

ISBLANK1        LDAA #' '               ; It's blank
                STAA TEN_THOUS          ;so store a space
                BRA  C_THOU             ;and check the ?thousands? digit

NOT_BLANK1      LDAA TEN_THOUS          ;Get the ?ten_thousands? digit
                ORAA #$30               ;Convert to ascii
                STAA TEN_THOUS
                LDAA #$1                ;Signal that we have seen a ?non-blank? digit
                STAA NO_BLANK

C_THOU          LDAA THOUSANDS          ;Check the thousands digit for blankness
                ORAA NO_BLANK           ;If it?s blank and ?no-blank? is still zero
                BNE  NOT_BLANK2

ISBLANK2        LDAA  #' '              ; Thousands digit is blank
                STAA THOUSANDS          ;so store a space
                BRA  C_HUNS             ;and check the hundreds digit

NOT_BLANK2      LDAA THOUSANDS          ;(similar to ?ten_thousands? case)
                ORAA #$30
                STAA THOUSANDS
                LDAA #$1
                STAA NO_BLANK

C_HUNS          LDAA HUNDREDS           ;Check the hundreds digit for blankness
                ORAA NO_BLANK           ;If it?s blank and ?no-blank? is still zero
                BNE NOT_BLANK3

ISBLANK3        LDAA  #' '              ; Hundreds digit is blank
                STAA HUNDREDS           ;so store a space
                BRA C_TENS              ;and check the tens digit

NOT_BLANK3      LDAA HUNDREDS           ;(similar to ?ten_thousands? case)
                ORAA #$30
                STAA HUNDREDS
                LDAA #$1
                STAA NO_BLANK

C_TENS          LDAA TENS               ;Check the tens digit for blankness
                ORAA NO_BLANK           ;If it?s blank and ?no-blank? is still zero
                BNE NOT_BLANK4  

ISBLANK4        LDAA  #' '              ; Tens digit is blank
                STAA TENS               ;so store a space
                BRA C_UNITS             ;and check the units digit

NOT_BLANK4      LDAA TENS               ;(similar to ?ten_thousands? case)
                ORAA #$30     
                STAA TENS

C_UNITS         LDAA UNITS              ;No blank check necessary, convert to ascii.
                ORAA #$30
                STAA UNITS

                RTS ;We?re done

;*****************************************************************
;Display the Battery Voltage
;*****************************************************************
                LDAA        #$C7        ; Move LCD Cursor to the second row
                JSR         cmd2LCD   
                LDAB        CRNT_STATE  ; Display Current State
                LSLB
                LSLB
                LSLB
                LDX         #tab
                ABX         
                JSR         putsLCD
                RTS

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
                ORG         $FFFE
                DC.W        Entry       ; Reset Vector       
                ORG         $FFDE
                DC.W        TOF_ISR     ; Timer Overflow Interrupt Vector