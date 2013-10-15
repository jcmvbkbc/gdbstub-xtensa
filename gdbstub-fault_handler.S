	.section ".KernelExceptionVector.text", "ax"
_KernelExceptionVector:
	j	1f
	.literal_position
1:
	xsr	a2, epc1
	addi	a2, a2, 3
	xsr	a2, epc1
	wsr	a2, excsave1
	movi	a2, fault_handler
	jx	a2
	
	.section ".UserExceptionVector.text", "ax"
_UserExceptionVector:
	j	1f
	.literal_position
1:
	xsr	a2, epc1
	addi	a2, a2, 3
	xsr	a2, epc1
	wsr	a2, excsave1
	movi	a2, fault_handler
	jx	a2

	.text
	.global	fault_handler
	.align	4
fault_handler:
	movi	a2, mem_err
	s32i	a2, a2, 0
	rsr	a2, excsave1
	rfe
