.origin 0

.macro NOP
NOP:
        ADD     r0, r0, 0
.endm

START:

	CLR R30.t10 // clock edge
    //MOV var1, r31.w0 // read inputs
    NOP
    NOP
    SET R30.t10 // clock edge
    NOP
	QBA START
