/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_angular_drone.h"

#define NX     ANGULAR_DRONE_NX
#define NZ     ANGULAR_DRONE_NZ
#define NU     ANGULAR_DRONE_NU
#define NP     ANGULAR_DRONE_NP


int main()
{
    int status = 0;
    sim_solver_capsule *capsule = angular_drone_acados_sim_solver_create_capsule();
    status = angular_drone_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = angular_drone_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = angular_drone_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = angular_drone_acados_get_sim_out(capsule);
    void *acados_sim_dims = angular_drone_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;
    x_current[13] = 0.0;
    x_current[14] = 0.0;
    x_current[15] = 0.0;
    x_current[16] = 0.0;
    x_current[17] = 0.0;
    x_current[18] = 0.0;
    x_current[19] = 0.0;
    x_current[20] = 0.0;
    x_current[21] = 0.0;
    x_current[22] = 0.0;
    x_current[23] = 0.0;
    x_current[24] = 0.0;
    x_current[25] = 0.0;
    x_current[26] = 0.0;

  
    x_current[0] = -0.0004770002996478912;
    x_current[1] = 0.000040441542396178327;
    x_current[2] = 0.4662533658816522;
    x_current[3] = 0.33258307957631006;
    x_current[4] = 0.3632048083535026;
    x_current[5] = 0.332641866560553;
    x_current[6] = 0.36324102458932456;
    x_current[7] = 0.4373768148269408;
    x_current[8] = 0.0433022054628637;
    x_current[9] = 0.000035618585264478974;
    x_current[10] = -0.000004651240908204457;
    x_current[11] = 1.2414427414304696;
    x_current[12] = 0.005024915769352508;
    x_current[13] = -0.0023964333699890566;
    x_current[14] = 0.00007389987938181962;
    x_current[15] = -0.00000000016567062084067718;
    x_current[16] = -0.000004651240905287522;
    x_current[17] = 0.9999999993656582;
    x_current[18] = 0.00003561858525694752;
    x_current[19] = 0.00003561858525733281;
    x_current[20] = 0.9999999993764752;
    x_current[21] = -0.000012041875831060585;
    x_current[22] = 0.0000003713406692589538;
    x_current[23] = -0.0000001770961369887588;
    x_current[24] = 0.00003219978624260007;
    x_current[25] = -0.000015921413994864085;
    x_current[26] = 0.9999999993548412;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        status = angular_drone_acados_sim_solve(capsule);

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = angular_drone_acados_sim_free(capsule);
    if (status) {
        printf("angular_drone_acados_sim_free() returned status %d. \n", status);
    }

    angular_drone_acados_sim_solver_free_capsule(capsule);

    return status;
}
