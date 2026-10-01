function delta_send(s, mode, payload)
% delta_send — send one transaction in the GUI's framing: <mode><{json}><#>
msg = sprintf('<%c><%s><#>', mode, jsonencode(payload));
write(s, msg, "char");
end
