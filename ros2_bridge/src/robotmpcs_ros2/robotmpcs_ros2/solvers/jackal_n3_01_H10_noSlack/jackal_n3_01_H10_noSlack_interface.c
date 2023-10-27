/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#include "include/jackal_n3_01_H10_noSlack.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "jackal_n3_01_H10_noSlack_model.h"



/* copies data from sparse matrix into a dense one */
static void jackal_n3_01_H10_noSlack_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, jackal_n3_01_H10_noSlack_callback_float *data, jackal_n3_01_H10_noSlack_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((jackal_n3_01_H10_noSlack_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default jackal_n3_01_H10_noSlack_adtool2forces(jackal_n3_01_H10_noSlack_float *x,        /* primal vars                                         */
                                 jackal_n3_01_H10_noSlack_float *y,        /* eq. constraint multiplers                           */
                                 jackal_n3_01_H10_noSlack_float *l,        /* ineq. constraint multipliers                        */
                                 jackal_n3_01_H10_noSlack_float *p,        /* parameters                                          */
                                 jackal_n3_01_H10_noSlack_float *f,        /* objective function (scalar)                         */
                                 jackal_n3_01_H10_noSlack_float *nabla_f,  /* gradient of objective function                      */
                                 jackal_n3_01_H10_noSlack_float *c,        /* dynamics                                            */
                                 jackal_n3_01_H10_noSlack_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 jackal_n3_01_H10_noSlack_float *h,        /* inequality constraints                              */
                                 jackal_n3_01_H10_noSlack_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 jackal_n3_01_H10_noSlack_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
                                 solver_int32_default iteration, /* iteration number of solver                         */
                                 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const jackal_n3_01_H10_noSlack_callback_float *in[4];
    jackal_n3_01_H10_noSlack_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    jackal_n3_01_H10_noSlack_callback_float w[26];
	
    /* temporary storage for AD tool sparse output */
    jackal_n3_01_H10_noSlack_callback_float this_f = (jackal_n3_01_H10_noSlack_callback_float) 0.0;
    jackal_n3_01_H10_noSlack_float nabla_f_sparse[5];
    jackal_n3_01_H10_noSlack_float h_sparse[11];
    jackal_n3_01_H10_noSlack_float nabla_h_sparse[13];
    jackal_n3_01_H10_noSlack_float c_sparse[1];
    jackal_n3_01_H10_noSlack_float nabla_c_sparse[1];
    
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 8))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		jackal_n3_01_H10_noSlack_objective_0(in, out, NULL, w, 0);
		if( nabla_f != NULL )
		{
			nrow = jackal_n3_01_H10_noSlack_objective_0_sparsity_out(1)[0];
			ncol = jackal_n3_01_H10_noSlack_objective_0_sparsity_out(1)[1];
			colind = jackal_n3_01_H10_noSlack_objective_0_sparsity_out(1) + 2;
			row = jackal_n3_01_H10_noSlack_objective_0_sparsity_out(1) + 2 + (ncol + 1);
				
			jackal_n3_01_H10_noSlack_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		jackal_n3_01_H10_noSlack_rktwo_0(x, p, c, nabla_c, jackal_n3_01_H10_noSlack_cdyn_0rd_0, jackal_n3_01_H10_noSlack_cdyn_0, threadID);
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		jackal_n3_01_H10_noSlack_inequalities_0(in, out, NULL, w, 0);
		if( h != NULL )
		{
			nrow = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(0)[0];
			ncol = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(0)[1];
			colind = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(0) + 2;
			row = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
				
			jackal_n3_01_H10_noSlack_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h != NULL )
		{
			nrow = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(1)[0];
			ncol = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(1)[1];
			colind = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(1) + 2;
			row = jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
				
			jackal_n3_01_H10_noSlack_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((9 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		jackal_n3_01_H10_noSlack_objective_1(in, out, NULL, w, 0);
		if( nabla_f != NULL )
		{
			nrow = jackal_n3_01_H10_noSlack_objective_1_sparsity_out(1)[0];
			ncol = jackal_n3_01_H10_noSlack_objective_1_sparsity_out(1)[1];
			colind = jackal_n3_01_H10_noSlack_objective_1_sparsity_out(1) + 2;
			row = jackal_n3_01_H10_noSlack_objective_1_sparsity_out(1) + 2 + (ncol + 1);
				
			jackal_n3_01_H10_noSlack_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		jackal_n3_01_H10_noSlack_inequalities_1(in, out, NULL, w, 0);
		if( h != NULL )
		{
			nrow = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(0)[0];
			ncol = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(0)[1];
			colind = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(0) + 2;
			row = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
				
			jackal_n3_01_H10_noSlack_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h != NULL )
		{
			nrow = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(1)[0];
			ncol = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(1)[1];
			colind = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(1) + 2;
			row = jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
				
			jackal_n3_01_H10_noSlack_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((jackal_n3_01_H10_noSlack_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
