% Graphing for k-connectivity tests

B = []
for i=0:99
    fname = int2str(i)+'/blind.csv';
   b = load(fname);
   B = [B;mean(b)];
end