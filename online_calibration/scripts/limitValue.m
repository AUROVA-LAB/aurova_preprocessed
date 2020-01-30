function output_value = limitValue(input_value, dw_limit, up_limit)

output_value = input_value;

if input_value < dw_limit
    output_value = dw_limit;
end

if input_value > up_limit
    output_value = up_limit;
end

end

