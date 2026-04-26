function out = merge_structs(base, override)
%MERGE_STRUCTS Recursively merge two structs.

if nargin < 1 || isempty(base)
    base = struct();
end
if nargin < 2 || isempty(override)
    out = base;
    return;
end

out = base;
fields = fieldnames(override);

for i = 1:numel(fields)
    name = fields{i};
    value = override.(name);

    if isstruct(value) && isfield(out, name) && isstruct(out.(name))
        out.(name) = merge_structs(out.(name), value);
    else
        out.(name) = value;
    end
end
