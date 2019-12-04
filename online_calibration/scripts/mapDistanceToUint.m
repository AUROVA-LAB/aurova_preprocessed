function uint_value = mapDistanceToUint(dist_value, sigma, base)

  dist_value = dist_value * 100.0 + 200.0; %conversion to cm and displacement
  sigma = sigma * 100.0;
  ini = 0.0;

  normalization_factor = base / ((1 / (sqrt(2 * pi) * sigma)) * exp(-(pow2(ini, 2) / (2 * pow2(sigma, 2)))));
  exp_value = (1 / (sqrt(2 * pi) * sigma)) * exp(-(pow2(dist_value, 2) / (2 * pow2(sigma, 2))));

  uint_value = uint8(exp_value * normalization_factor);

end

