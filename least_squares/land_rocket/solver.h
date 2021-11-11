#pragma once

// Unconstrained problem
#include "solver_naive.h"
#include "solver_update_ata_in_place.h"
#include "solver_sparse.h"
#include "solver_gradient_desent.h"

// Constrained problem
#include "solver_primal_dual.h"
#include "solver_primal_dual_structural.h"

// DDP
#include "solver_ddp.h"