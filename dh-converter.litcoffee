Converting from D-H notation to URDF files
----------------------


This file converts files in format of Markdown table into a URDF (universal robot description format) file representing of kinematic chain.
Tables should have a format of, with R being whether a joint is Rotational (true| false). Header is not nessessary.

| theta | d | a | alpha |R  |
|-------|---|---|-------|---|
|       |   |   |       |   |
|       |   |   |       |   |
|       |   |   |       |   |

Generally as this notation translates into series of transformations Rot(z,theta)Trans(z,d)Trans(x,a)Rot(x,alpha) two joints -- active and fixed are needed
for each row of the table. 
    
    window.DH_converter = window.DH_converter||{};    

    class window.Robot_Maker
        
        constructor: (@table) ->
            @parser= new DOMParser();
            @robot_dom= @parser.parseFromString(@robot_xml,"text/xml");
            @robot_dom_modified= @parser.parseFromString(@robot_xml_modified,"text/xml");
            @urdf=@convert_text_table(@table)
        robot_xml : "<robot name='example_robot'>
            <link name='link0_passive'><visual>
            <material name='blue'><color rgba='0 0 .8 1'/></material>
            <geometry> <origin xyz='0 0 0' rpy='0 0 0'/>
            
            <cylinder length='0.0' radius='0.0'/></geometry></visual></link>
            
            <link name='link0_x_axis'><visual>
             <material name='red'><color rgba='1 0 0 1'/></material>
            <geometry> <origin xyz='0 0 0' rpy='0 0 0'/>
            <cylinder length='0.0' radius='0.0'/></geometry></visual></link>
            <joint name='q0_x' type='fixed'>
            <origin xyz='0 0 0' rpy='0 1.571 0'/>
            <parent link='link0_passive' />
            <child link='link0_x_axis' />
            </joint>
            
            </robot> ";
        robot_xml_modified : "<robot name='example_robot'>
            <link name='link0'><visual>
            <material name='blue'><color rgba='0 0 .8 1'/></material>
            <geometry> <origin xyz='0 0 0' rpy='0 0 0'/>
            
            <cylinder length='0.6' radius='0.1'/></geometry></visual></link>
            
            
            </robot> ";

A Moustache template is prepared to put the adequate values from the row od DH table

        row_template_insert:"
        <link name='link{{name}}'></link>
        <link name='link{{name}}_x_axis'><visual><origin xyz='0 0 0.25' rpy='0 0 0'/> <material name='red' /><geometry><cylinder length='0.5' radius='0.05'/></geometry></visual></link>
        
    
        <joint name='q{{name}}' type='{{type}}'>
        <origin xyz='0 0 {{d}}' rpy='0 0 {{th}}'/>
        <parent link='link{{previous_name}}_passive' />
        <child link='link{{name}}' />
        <axis xyz='0 0 1'/>
        </joint>
        
        
        <joint name='q{{row_no}}_passive' type='fixed'>
        <origin xyz='{{a}} 0 0' rpy='{{alpha}} 0 0'/>
        <parent link='link{{name}}' />
        <child link='link{{name}}_passive' />
        </joint>";
        row_template_add_x:"<link name='link{{name}}_passive'><visual> <origin xyz='0 0 0.25' rpy='0 0 0'/><material name='blue' /><geometry><cylinder length='0.5' radius='0.05'/></geometry></visual></link>
        <joint name='q{{row_no}}_x' type='fixed'>
        <origin xyz='0 0 0' rpy='0 1.571 0'/>
        <parent link='link{{name}}_passive' />
        <child link='link{{name}}_x_axis' />
        </joint>
        ";
        modified_dh_row_template_insert:"<link name='link{{name}}'>
        <visual><origin xyz='0 0 0.25' rpy='0 0 0'/> <material name='red' />
        <geometry><cylinder length='0.5' radius='0.05'/></geometry></visual></link>
        
        <joint name='q{{name}}' type='{{type}}'>
        <origin xyz='{{a}} {{dy}} {{dz}}' rpy='{{alpha}} 0 {{th}}'/>
        <parent link='link{{previous_name}}' />
        <child link='link{{name}}' />
        <axis xyz='0 0 1'/>
        </joint>
        
        
        
        "    


Using regex, we parse rows of table, starting from 2nd row. Regex looks for floating values for every but last column, we convert last column to boolean value.
Then, each row is passed to DH_row_to_links function

        convert_text_table: (table) ->
    
            ### function converts markdown table into nodes, we assume that table has header (two lines)
            ###



            
            lines_of_text=table.split('\n');
            robot_dict={}
            
First we look whether user is using DH parameters or modified DH parameters, what is the sequence of columns of the first row

            standard_dh=new RegExp(/\|(th|theta)\|d\|a\|alpha\|R\|/i)
            modifed_dh=new RegExp(/\|a\|alpha\|(th|theta)\|d\|R\|/i)
            
            
            
            is_standard=standard_dh.test(lines_of_text[0])
            is_modified=modifed_dh.test(lines_of_text[0])

            if is_standard
                sam_robot = @robot_dom.getElementsByTagName("robot")[0]
            else if is_modified
                sam_robot = @robot_dom_modified.getElementsByTagName("robot")[0]
            else
                                window.alert("Not a valid table. please use table header in form of |th|d|a|alpha|R| or |a|alpha|th|d|R|");
                                return null


                
                
A pattern is created to seek for valid DH values -- that is floating point numbers that are preeceeded by pipe |

            #pattern=/\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?(\w+?)\W*?\|/;
            #regg=new RegExp(pattern)
We hardcode that every line from the 3rd is treated as a row containing DH-values

            for line_no in [2...lines_of_text.length]
                 robot_dict.row_no=line_no-1
                 
                 #przerob_linijke=lines_of_text[line_no].match(regg)
                 lines_of_text[line_no].replace(/\s/g, "");
                 przerob_linijke=lines_of_text[line_no].split('|')
                 if lines_of_text[line_no].length==0
                     continue
                 if przerob_linijke==null or przerob_linijke.length<5
                     window.alert("line "+(line_no-1)+" is not valid") # TODO it should be some one nice alerting system
                     console.log("line "+(line_no-1)+" is not valid")
                     continue;
                 try
                     if is_standard    
                         robot_dict.th=math.eval(przerob_linijke[1])
                         robot_dict.d=math.eval(przerob_linijke[2])
                         robot_dict.a=math.eval(przerob_linijke[3])
                         robot_dict.alpha=math.eval(przerob_linijke[4])
                         robot_dict.R=przerob_linijke[5]=="true"
                     if is_modified
                         robot_dict.a=math.eval(przerob_linijke[1])    
                         robot_dict.alpha=math.eval(przerob_linijke[2])
                         robot_dict.th=math.eval(przerob_linijke[3])
                         robot_dict.d=math.eval(przerob_linijke[4])
                         robot_dict.modified_dh=true
                         robot_dict.R=przerob_linijke[5]=="true"
                     console.log(robot_dict)
                 catch e 
                     window.alert("line "+(line_no-1)+" is not valid") # TODO it should be some one nice alerting system
                     console.log("line "+(line_no-1)+" is not valid")
                     continue;
                 wynik=@DH_row_to_links(robot_dict)
                 
                 sam_robot.insertAdjacentHTML('beforeend',wynik)
                
            return vkbeautify.xml(sam_robot.outerHTML) #we beutify result using outside tool
            
Function below takes a dictionary of row values and converts it into xml code. 

    
        DH_row_to_links: (row_dict,add_x=true) ->
            ### function gets a dictionary representing row in DH table, returns xml node based on two links
            additional code representing x axis can be added if add_x is true
            ###
            
            row_no=row_dict.row_no
            row_dict.name=row_no
            row_dict.previous_name=row_no-1
            if row_dict.R
                row_dict.type="revolute"
            else
                row_dict.type="prismatic"
                
            template_xml=@row_template_insert
            
            if add_x
                template_xml=template_xml+@row_template_add_x
            
There is the option of using modified Denavit-Hartenberg parameters, where the sequence of transformations is different. 
Actually, this gives the opportunity to use only one link per row (while treating each row equally),
 but some math is needed to calculate the start translation between joints. 
That is we translation=[a;0;0]+ R[x,alpha] [0;0;d]

            if row_dict.modified_dh
                
                alpha=parseFloat(row_dict.alpha)
                d=parseFloat(row_dict.d)
                row_dict.dy=-Math.sin(alpha)*d
                row_dict.dz=Math.cos(alpha)*d
                console.log(row_dict.dy)
                console.log(row_dict.dz)
                template_xml=@modified_dh_row_template_insert
            
            
            row_xml=Mustache.render(template_xml,row_dict)
            return     row_xml
        


        
        
        
        
    
