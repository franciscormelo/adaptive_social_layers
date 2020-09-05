% -----------------------------------------
% Graph-Cuts for F-Formation (GCFF) for a single frame for python and ros
% integration
% -----------------------------------------
% Graph-Cuts for F-Formation (GCFF)
% 2015 - University of Verona
% Written by Francesco Setti
% -----------------------------------------

function groups = gcff(mdl, stride, persons)
    addpath ('graphopt')
    
    mdl = double(mdl);
    stride = double(stride);

    idx = 1:1:size(persons,1);
    persons_idx = [idx', persons];
    gg = gc( persons_idx, stride, mdl ) ;
    groups= [] ;
    
    for ii = 1:max(gg)+1
        groups{ii} = persons_idx(gg==ii-1,[2:end]) ;
    end
    
end


