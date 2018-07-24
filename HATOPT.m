function w_hat = HATOPT(w)
%HATOPT(w) takes 3-by-1 vector and output 3-by-3 matrix corresponding to
%hat operation
[rows, cols] = size(w);
if ((rows ~= 3) || (cols ~= 1))
  error('HATOPT requires a 3x1 vector argument. Check your dimensions.');
end

w_hat = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];

end

