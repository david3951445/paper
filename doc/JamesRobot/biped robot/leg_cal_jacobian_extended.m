clc;clear;
load 'Jb.mat'

Je = blkdiag(Jb_inv,eye(4));
Je_inv = inv(Je);
save Je.mat