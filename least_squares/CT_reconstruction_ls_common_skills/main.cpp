#include <iostream>

#include "ct_reconstruction.h"


int main(int argc, char *argv[])
{
    CtReconstruction ct_reconstruction;
    // "gradient_desent", "least_square_dense", "least_square_sparse"
    ct_reconstruction.params_.optimizer_type = "gradient_desent";

    ct_reconstruction.ct_reconstruction_simulation();

    return 0;
}