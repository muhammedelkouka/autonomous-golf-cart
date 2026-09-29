function S = kml2struct(kmlFile)
%KML2STRUCT  Parse KML file into a MATLAB struct
%
%   S = kml2struct(kmlFile)
%   - Handles both 2D (lon,lat) and 3D (lon,lat,alt) coordinates
%   - Returns struct array with fields: Geometry, Name, Lon, Lat, Alt

    % Read file
    if nargin < 1
        error('No KML file specified.');
    end
    if ~isfile(kmlFile)
        error('KML file not found: %s', kmlFile);
    end
    txt = fileread(kmlFile);

    % Parse XML
    xDoc = xmlreadstring(txt);

    % Find Placemark elements
    placemarks = xDoc.getElementsByTagName('Placemark');
    nPlacemarks = placemarks.getLength;

    % Initialize output struct
    S = repmat(struct('Geometry',[],'Name',[],'Lon',[],'Lat',[],'Alt',[]), nPlacemarks,1);

    for i = 1:nPlacemarks
        placemark = placemarks.item(i-1);

        % Name
        nameNode = placemark.getElementsByTagName('name');
        if nameNode.getLength > 0
            S(i).Name = char(nameNode.item(0).getFirstChild.getData);
        else
            S(i).Name = '';
        end

        % Geometry type
        if placemark.getElementsByTagName('LineString').getLength > 0
            S(i).Geometry = 'Line';
            coordsNode = placemark.getElementsByTagName('coordinates').item(0);
        elseif placemark.getElementsByTagName('Polygon').getLength > 0
            S(i).Geometry = 'Polygon';
            coordsNode = placemark.getElementsByTagName('coordinates').item(0);
        elseif placemark.getElementsByTagName('Point').getLength > 0
            S(i).Geometry = 'Point';
            coordsNode = placemark.getElementsByTagName('coordinates').item(0);
        else
            S(i).Geometry = 'Unknown';
            continue;
        end

        % Extract coordinates text
        coordStr = strtrim(char(coordsNode.getFirstChild.getData));
        coordStr = regexprep(coordStr,'\s+',' '); % clean spaces
        coordCells = strsplit(coordStr);

        % Convert each "lon,lat[,alt]" into numbers
        coords = cellfun(@(c) sscanf(c, '%f,%f,%f')', coordCells, 'UniformOutput', false);
        coords = vertcat(coords{:});

        % Ensure at least 3 columns
        if size(coords,2) == 2
            coords(:,3) = 0;
        end

        % Store
        S(i).Lon = coords(:,1);
        S(i).Lat = coords(:,2);
        S(i).Alt = coords(:,3);
    end
end


%% Helper: xmlreadstring
function xDoc = xmlreadstring(str)
    import java.io.*;
    import org.xml.sax.InputSource;
    import javax.xml.parsers.*;
    factory = DocumentBuilderFactory.newInstance;
    builder = factory.newDocumentBuilder;
    isrc = InputSource;
    isrc.setCharacterStream(StringReader(str));
    xDoc = builder.parse(isrc);
end
