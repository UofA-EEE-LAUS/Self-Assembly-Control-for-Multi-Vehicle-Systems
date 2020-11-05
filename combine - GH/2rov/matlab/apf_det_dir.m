function [ det_dist] = apf_det_dir(han)

C_H = [68,64,65,66,67,209];

if han == 68
    det_dist = 0.3+0.25;
elseif han == 64
    det_dist = 0.27+0.25;
elseif han == 65
    det_dist = 0.5+0.25;
elseif han == 66
    det_dist = 0.4+0.25;
elseif han == 67
    det_dist = 0.8+0.25;
elseif han == 209
    det_dist = 0.74+0.25;
else
    det_dist = 0.8+0.25;
end

det_dist
han

end