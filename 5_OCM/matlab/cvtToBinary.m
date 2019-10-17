measID = fopen('meas.bin','w');

s = size(z);
num_elem = prod(s);
for ii = 1:s(3)
   for jj = 1:s(2)
       for kk = 1:s(1)
           fwrite(measID,z(kk,jj,ii),'double');
       end
   end
end


fclose(measID);

thkID = fopen('thk.bin','w');

s = size(thk);
num_elem = prod(s);

for jj = 1:s(2)
   for kk = 1:s(1)
       fwrite(thkID,thk(kk,jj),'double');
   end
end

fclose(thkID);

stateID = fopen('state.bin','w');

s = size(X);
num_elem = prod(s);

for jj = 1:s(2)
   for kk = 1:s(1)
       fwrite(stateID,X(kk,jj),'double');
   end
end

fclose(stateID);

mapID = fopen('map.bin','w');

s = size(map);
num_elem = prod(s);

for jj = 1:s(2)
   for kk = 1:s(1)
       fwrite(mapID,map(kk,jj),'double');
   end
end

fclose(mapID);