function mu = manipulability( J, measure )
%Compute a measure of manipulability. Implement all three different types: 'sigmamin',
%'detjac', or 'invcond', as defined in Chapter 3, Section 4.4 [1]. This function will return
%any one of the three measures of manipulability as defined by the second argument (see
%below).
switch measure
    case 'sigmamin'
        mu = min(svd(J));
    case 'detjac'
        mu = det(J);
    case 'invcond'
        sigma_max = norm(J);
        sigma_min = min(svd(J));
        mu = sigma_min/sigma_max;
end

end

