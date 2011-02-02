#ifndef _A_STACKOP_
#define _A_STACKOP_

/* record for defining effect of stack operation */

typedef struct aTEADebugStackOp
{
  char cUpdateType;	/* how stack has changed after op */
  char cRefreshCt;	/* how many bytes must be refreshed after op */
  char cKnownSizeChg;	/* known change to stack */
  char cAuxChgType;	/* some stack size changes depend on data */
} aTEADebugStackOp;



/* modes */
#define	aSTACKMODE_REL	0
#define	aSTACKMODE_ABS	1
#define	aSTACKMODE_CHK	2


/* some port writes will cause a later alteration of the stack
/* (may need to check if SP is mismatched with internal SP)
/* when stack shrinks, leave previous crap in stack

/* values for cUpdateType */

#define	aCHGTO_NONE	0	/* update no bytes */
#define	aCHGTO_TOPN	1	/* update top bytes */
#define	aCHGTO_RELS	2	/* update bytes at rel offset */
#define	aCHGTO_ABSS	3	/* update bytes at abs address */
#define	aCHGTO_RELX	4	/* update bytes at rel offset on stack */
#define	aCHGTO_ABSX	5	/* update bytes at abs address on stack */

/* values for cAuxChgType */

#define	aSZDEP_ZERO	0	/* no additional change to stack size */
#define	aSZDEP_OPCP	1	/* opcode has change (+) */
#define	aSZDEP_OPCN	2	/* opcode has change (-) */
#define	aSZDEP_TOPP	3	/* top stack byte is change (+) */
#define	aSZDEP_TOPN	4	/* top stack byte is change (-) */
#define	aSZDEP_CONV	5	/* change based on "fmt" conversion */
#define	aSZDEP_MEMW	6	/* change depends on port operation */
#define	aSZDEP_EXIT	7	/* change depends on exit */

aTEADebugStackOp aTEADebugStackOpArray[84]=
{
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_NOP */
{aCHGTO_TOPN, 1,   +1, aSZDEP_ZERO}, /* op_PUSHLB */
{aCHGTO_TOPN, 2,   +2, aSZDEP_ZERO}, /* op_PUSHLS */
{aCHGTO_TOPN, 1,   +1, aSZDEP_ZERO}, /* op_PUSHMB */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_PUSHMBX */
{aCHGTO_TOPN, 2,   +2, aSZDEP_ZERO}, /* op_PUSHMS */
{aCHGTO_TOPN, 2,    0, aSZDEP_ZERO}, /* op_PUSHMSX */
{aCHGTO_TOPN, 1,   +1, aSZDEP_ZERO}, /* op_PUSHSB */
{aCHGTO_TOPN, 1,    0, aSZDEP_ZERO}, /* op_PUSHSBX */
{aCHGTO_TOPN, 2,   +2, aSZDEP_ZERO}, /* op_PUSHSS */
{aCHGTO_TOPN, 2,   +1, aSZDEP_ZERO}, /* op_PUSHSSX */
{aCHGTO_TOPN, 1,   +2, aSZDEP_ZERO}, /* op_PUSHSBA */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_PUSHSBAX */
{aCHGTO_TOPN, 2,   +2, aSZDEP_ZERO}, /* op_PUSHSSA */
{aCHGTO_TOPN, 2,    0, aSZDEP_ZERO}, /* op_PUSHSSAX */
{aCHGTO_TOPN,-1,    0, aSZDEP_OPCP}, /* op_PUSHN */
{aCHGTO_TOPN,-1,   -1, aSZDEP_TOPP}, /* op_PUSHNX */

{aCHGTO_TOPN, 2,   +1, aSZDEP_ZERO}, /* op_CONVBS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_CONVSB */

{aCHGTO_NONE, 0,   -1, aSZDEP_MEMW}, /* op_POPBM */
{aCHGTO_NONE, 0,   -3, aSZDEP_MEMW}, /* op_POPBMX */
{aCHGTO_NONE, 0,   -2, aSZDEP_MEMW}, /* op_POPSM */
{aCHGTO_NONE, 0,   -4, aSZDEP_MEMW}, /* op_POPSMX */
{aCHGTO_RELS, 1,   -1, aSZDEP_ZERO}, /* op_POPBS */
{aCHGTO_RELX, 1,   -2, aSZDEP_ZERO}, /* op_POPBSX */
{aCHGTO_RELS, 2,   -2, aSZDEP_ZERO}, /* op_POPSS */
{aCHGTO_RELX, 2,   -3, aSZDEP_ZERO}, /* op_POPSSX */
{aCHGTO_ABSS, 1,   -1, aSZDEP_ZERO}, /* op_POPBSA */
{aCHGTO_ABSX, 1,   -3, aSZDEP_ZERO}, /* op_POPBSAX */
{aCHGTO_ABSS, 2,   -2, aSZDEP_ZERO}, /* op_POPSSA */
{aCHGTO_ABSX, 2,   -4, aSZDEP_ZERO}, /* op_POPSSAX */
{aCHGTO_NONE, 0,    0, aSZDEP_TOPN}, /* op_POPCMD */
{aCHGTO_NONE, 0,   -1, aSZDEP_ZERO}, /* op_POPB */
{aCHGTO_NONE, 0,   -2, aSZDEP_ZERO}, /* op_POPS */
{aCHGTO_NONE, 0,    0, aSZDEP_OPCN}, /* op_POPN */
{aCHGTO_NONE, 0,   -1, aSZDEP_TOPN}, /* op_POPNX */

{aCHGTO_RELS, 1,    0, aSZDEP_ZERO}, /* op_DECB */
{aCHGTO_RELS, 2,    0, aSZDEP_ZERO}, /* op_DECS */
{aCHGTO_RELS, 1,    0, aSZDEP_ZERO}, /* op_INCB */
{aCHGTO_RELS, 2,    0, aSZDEP_ZERO}, /* op_INCS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_ADDB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_ADDS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_SUBB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_SUBS */
{aCHGTO_TOPN, 1,    0, aSZDEP_ZERO}, /* op_NEGB */
{aCHGTO_TOPN, 2,    0, aSZDEP_ZERO}, /* op_NEGS */
{aCHGTO_TOPN, 2,    0, aSZDEP_ZERO}, /* op_MULTB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_MULTS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_DIVB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_DIVS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_MODB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_MODS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_ANDB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_ANDS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_ORB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_ORS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_XORB */
{aCHGTO_TOPN, 2,   -2, aSZDEP_ZERO}, /* op_XORS */
{aCHGTO_TOPN, 1,    0, aSZDEP_ZERO}, /* op_COMPB */
{aCHGTO_TOPN, 2,    0, aSZDEP_ZERO}, /* op_COMPS */

{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_RLB */
{aCHGTO_TOPN, 2,   -1, aSZDEP_ZERO}, /* op_RLS */
{aCHGTO_TOPN, 1,   -1, aSZDEP_ZERO}, /* op_RRB */
{aCHGTO_TOPN, 2,   -1, aSZDEP_ZERO}, /* op_RRS */

{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_BRNEG */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_BRPOS */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_BRZ */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_BRNZ */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_BRC */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_BRNC */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_CMPBBR */
{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_CMPSBR */

{aCHGTO_NONE, 0,    0, aSZDEP_ZERO}, /* op_GOTO */

{aCHGTO_TOPN, 2,   +2, aSZDEP_ZERO}, /* op_CALL */
{aCHGTO_NONE, 0,   -2, aSZDEP_ZERO}, /* op_RETURN */

{aCHGTO_TOPN,-1,   -1, aSZDEP_CONV}, /* op_FMTBB */
{aCHGTO_TOPN,-1,   -1, aSZDEP_CONV}, /* op_FMTBD */
{aCHGTO_TOPN,-1,   -1, aSZDEP_CONV}, /* op_FMTBU */
{aCHGTO_TOPN,-1,   -1, aSZDEP_CONV}, /* op_FMTBH */
{aCHGTO_TOPN,-1,   -2, aSZDEP_CONV}, /* op_FMTSB */
{aCHGTO_TOPN,-1,   -2, aSZDEP_CONV}, /* op_FMTSD */
{aCHGTO_TOPN,-1,   -2, aSZDEP_CONV}, /* op_FMTSU */
{aCHGTO_TOPN,-1,   -2, aSZDEP_CONV}, /* op_FMTSH */

{aCHGTO_NONE,-1,    0, aSZDEP_EXIT}  /* op_EXIT */
};

#endif /* _A_STACKOP_ */
