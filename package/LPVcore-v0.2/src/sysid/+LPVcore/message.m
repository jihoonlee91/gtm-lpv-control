function msgobj = message(msgid, varargin)
% LPVCORE.MESSAGE is a LPVcore internal utility to access translated
%   message strings. MSGOBJ = LPVCORE.MESSAGE(MSGID,PARAMS) constructs an 
%   object that includes localized text associated with identifier MSGID.
%
%   The MESSAGE function searches only LPVcore message catalogs.
%
%   See also LPVCORE.ERROR, LPVCORE.MESSAGESTR, MESSAGE, CTRLMSGUTILS.ERROR

%   This function is closely related to message. However, custom toolboxes
%   cannot uses the message catalog in MATLAB.
%
%   The variable arguments args are used to fill in the predefined 
%   holes in the message string. The XML catalog message library can be
%   found in resources/en/
%
%
%   The XML sheet should consist of one three called 'message' with
%   children called 'entry'. The entry should have the item called 'key'.
%   The message itself is processed via sprinft and special characters
%   should be handled accordingly. 
%   Additionally, it is possible to include PARAMS in the string by using
%   {1}, {2}, {3}, ... corresponding to the input number. Formating of the
%   passed PARAMS can be done by:
%     - {#}                (Default) Seen as string. Equivalent to %s in
%                          sprintf.
%     - {#,number}         Seen as number. Equivalent to %g in sprintf.
%     - {#,number,exp}     Format as exponent. Equivalent to %e in sprintf.
%     - {#,number,integer} Format as integer. Equivalent to %d in sprintf.
%
%   Example: 
%
%   Create following entry in general.xml.
%
%   <entry key="testMsg">Prop. ''{1}'' is not a {2,number,integer} dimensional vector. Type "help {3}" for more information.</entry>
%
%   
%   >> LPVcore.message('LPVcore:general:testMsg', 'Myprop', 5, 'LPVcore')
%   
%   and = 
%
%   struct with fields:
%   
%     identifier: 'LPVcore:general:testMsg'
%        message: 'Prop. ''Myprop'' is not a 5 dimensional vector. Type "help LPVcore" for more information.'
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
    
    % Get LPVcore package directory
    [path,~,~] = fileparts(mfilename('fullpath'));

    msgid_s = split(msgid,':');
    
    if ~isequal(size(msgid_s),[3 1])
        throwAsCaller(MException('LPVcore:message:InvalidIdentifier', '%s', ...
            'Invalid MATLAB message identifier in the LPVcore library.  For more information, read about MException Identifiers in the MATLAB help.'));
    end
    
    
    xmlfile = [path, filesep , 'resources', filesep , 'en', filesep ,msgid_s{2}, '.xml'];
    
    try
        xml = xmlread(xmlfile);
    catch
        throwAsCaller(MException('LPVcore:message:UnknownCatalog', '%s', ...
            sprintf('Unable to load the LPVcore message catalog ''%s''. Please check the file location and format.',msgid_s{2})));
    end

    entry_list = xml.getElementsByTagName('entry');
    
    message = [];

    for k = 0:entry_list.getLength-1   % The Node is implemented in JAVA, index starts with 0
        
       ent_ = entry_list.item(k);
       key = char(ent_.getAttributes.getNamedItem('key').getNodeValue);

       if strcmp(key, msgid_s{3})
            att = ent_.getChildNodes;
            message = char(att.item(0).getNodeValue);
            break;
       end
    end
    
    if isempty(message)
        throwAsCaller(MException(msgid, '%s', ...
            sprintf('Unable to find message key ''%s'' in catalog ''LPVcore:%s''.',msgid_s{3},msgid_s{2})));
    end
    
    msgobj.identifier = msgid;
    
    if ~isempty(varargin)
        message = regexprep(message, ...
            {'{(\d+)}', '{(\d+),number}', '{(\d+),number,exp}', '{(\d+),number,integer}'}, ...
            {'\%$1\$s', '\%$1\$g',        '\%$1\$e',            '\%$1\$d'} );
        msgobj.message = sprintf(message, varargin{:});
    else
        msgobj.message = message;
    end
    
end