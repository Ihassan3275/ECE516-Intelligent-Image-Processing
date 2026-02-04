#include <iostream>
#include <octave/oct.h>

// DEFUN_DLD is still the correct macro for dynamically linked functions
DEFUN_DLD (comparagram, args, nargout,
  "AB = comparagram(A, B)\n\
  Generates a 256x256 joint histogram (comparagram) of two matrices.\n\
  Example: C = comparagram(image1, image2);")
{
  int nargin = args.length();
  
  // Basic validation
  if (nargin != 2)
  {
    print_usage();
    return octave_value_list();
  }

  // Use octave_value for input
  octave_value arg0 = args(0);
  octave_value arg1 = args(1);

  // Modern way to check for empty or mismatched dimensions
  if (arg0.isempty() || arg1.isempty())
    return octave_value(Matrix());

  if (arg0.rows() != arg1.rows() || arg0.columns() != arg1.columns())
  {
    // Modern error() accepts format strings directly without sprintf
    error("comparagram: input sizes %dx%d and %dx%d do not match",
          (int)arg0.rows(), (int)arg0.columns(), 
          (int)arg1.rows(), (int)arg1.columns());
    return octave_value_list();
  }

  if (arg0.is_real_matrix() && arg1.is_real_matrix())
  {
    // Convert inputs to Matrix (double precision)
    Matrix a = arg0.matrix_value();
    Matrix b = arg1.matrix_value();
    
    // Initialize a 256x256 matrix with zeros
    Matrix c(256, 256, 0.0); 

    for (octave_idx_type m = 0; m < a.rows(); m++) 
    {
      for (octave_idx_type n = 0; n < a.columns(); n++) 
      {
        // Extract values and cast to integer for indexing
        // Note: Assumes input values are in range 0-255
        int val_a = static_cast<int>(a(m, n));
        int val_b = static_cast<int>(b(m, n));

        if (val_a >= 0 && val_a < 256 && val_b >= 0 && val_b < 256)
        {
          c(val_a, val_b)++;
        }
      }
    }

    return octave_value(c);
  }
  else 
  {
    error("comparagram: inputs must be real matrices");
    return octave_value_list();
  }
}