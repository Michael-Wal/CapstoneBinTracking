# function [out] = findbin(x,y,z)
def findBin(x, y, z):

    #  section = {{-5,-3.75},{-6,-3},{0 1},'A1','Sec1','Shelf1';
    #             {-5,-3.75},{-6,-3},{1 2},'A1','Sec1','Shelf2';
    #             {-5,-3.75},{-6,-3},{2 3},'A1','Sec1','Shelf3';
    #             {-5,-3.75},{-6,-3},{3 4},'A1','Sec1','Shelf4';
    #             {-5,-3.75} {-3,0} {0,1},'A1', 'Sec2','Shelf1';
    #             {-5,-3.75} {-3,0} {1,2},'A1', 'Sec2','Shelf2';
    #             {-5,-3.75} {-3,0} {2 3},'A1', 'Sec2','Shelf3';
    #             {-5,-3.75} {-3,0} {3 4},'A1', 'Sec2','Shelf4';
    #             {-5,-3.75} {0,3} {0,1},'A1', 'Sec3','Shelf1';
    #             {-5,-3.75} {0,3} {1 2},'A1', 'Sec3','Shelf2';
    #             {-5,-3.75} {0,3} {2 3},'A1', 'Sec3','Shelf3';
    #             {-5,-3.75} {0,3} {3 4},'A1', 'Sec3','Shelf4';
    #             {-5,-3.75} {3,6} {0,1},'A1', 'Sec4','Shelf1';
    #             {-5,-3.75} {3,6} {1 2},'A1', 'Sec4','Shelf2';
    #             {-5,-3.75} {3,6} {2 3},'A1', 'Sec4','Shelf3';
    #             {-5,-3.75} {3,6} {3 4},'A1', 'Sec4','Shelf4'};
    #  save newinfo.mat section
        section = [
            ((-5, -3.75), (-6, -3), (0, 1), 'A1', 'Sec1', 'Shelf1'),
            ((-5, -3.75), (-6, -3), (1, 2), 'A1', 'Sec1', 'Shelf2'),
            ((-5, -3.75), (-6, -3), (2, 3), 'A1', 'Sec1', 'Shelf3'),
            ((-5, -3.75), (-6, -3), (3, 4), 'A1', 'Sec1', 'Shelf4'),
            ((-5, -3.75), (-3, 0), (0, 1), 'A1', 'Sec2', 'Shelf1'),
            ((-5, -3.75), (-3, 0), (1, 2), 'A1', 'Sec2', 'Shelf2'),
            ((-5, -3.75), (-3, 0), (2, 3), 'A1', 'Sec2', 'Shelf3'),
            ((-5, -3.75), (-3, 0), (3, 4), 'A1', 'Sec2', 'Shelf4'),
            ((-5, -3.75), (0, 3), (0, 1), 'A1', 'Sec3', 'Shelf1'),
            ((-5, -3.75), (0, 3), (1, 2), 'A1', 'Sec3', 'Shelf2'),
            ((-5, -3.75), (0, 3), (2, 3), 'A1', 'Sec3', 'Shelf3'),
            ((-5, -3.75), (0, 3), (3, 4), 'A1', 'Sec3', 'Shelf4'),
            ((-5, -3.75), (3, 6), (0, 1), 'A1', 'Sec4', 'Shelf1'),
            ((-5, -3.75), (3, 6), (1, 2), 'A1', 'Sec4', 'Shelf2'),
            ((-5, -3.75), (3, 6), (2, 3), 'A1', 'Sec4', 'Shelf3'),
            ((-5, -3.75), (3, 6), (3, 4), 'A1', 'Sec4', 'Shelf4'),
        ]
            
    # %building th shelves
    # %gm = multicuboid([7.5 10],[12 13],4);
    # gm = multicuboid([7.5 20],[12 13],4);
    # model = createpde;
    # model.Geometry = gm;
    # pdegplot(model,'CellLabels','off','EdgeLabels','off','FaceLabels','off','VertexLabels','off','FaceAlpha',0.5)
    "^ Plotting, not needed"

    # p1 = [-5 -6 1]; p11 = [5 -6 1];
    # p2 = [-3.75 -6 1]; p21 = [3.75 -6 1];
    # p3 = [-3.75 6 1];p31 = [3.75 6 1];
    # p4 = [-5 6 1]; p41 = [5 6 1]; 
    p1 = [-5, -6, 1]
    p2 = [-3.75, -6, 1]
    p3 = [-3.75, 6, 1]
    p4 = [-5, 6, 1]
    p11 = [5, -6, 1]
    p21 = [3.75, -6, 1]
    p31 = [3.75, 6, 1]
    p41 = [5, 6, 1]

    # x1 = [p1(1) p2(1) p3(1) p4(1)]; x11=[p11(1) p21(1) p31(1) p41(1)];
    # y1 = [p1(2) p2(2) p3(2) p4(2)]; y11 = [p11(2) p21(2) p31(2) p41(2)];
    # z1 = [p1(3) p2(3) p3(3) p4(3)]; z11 = [p11(3) p21(3) p31(3) p41(3)];
    x1 = [p1[0], p2[0], p3[0], p4[0]]
    x11 = [p11[0], p21[0], p31[0], p41[0]]
    y1 = [p1[1], p2[1], p3[1], p4[1]]
    y11 = [p11[1], p21[1], p31[1], p41[1]]
    z1 = [p1[2], p2[2], p3[2], p4[2]]
    z11 = [p11[2], p21[2], p31[2], p41[2]]

    # p5 = [-5 -3 0]; p51 = [5 -3 0];
    # p6 = [-3.75 -3 0]; p61 = [3.75 -3 0];
    # p7 = [-3.75 -3 4]; p71 = [3.75 -3 4];
    # p8 = [-5 -3 4]; p81 = [5 -3 4]; 
    p5 = [-5, -3, 0]
    p51 = [5, -3, 0]
    p6 = [-3.75, -3, 0]
    p61 = [3.75, -3, 0]
    p7 = [-3.75, -3, 4]
    p71 = [3.75, -3, 4]
    p8 = [-5, -3, 4]
    p81 = [5, -3, 4]

    # x2 = [p5(1) p6(1) p7(1) p8(1)]; x21 = [p51(1) p61(1) p71(1) p81(1)];
    # y2 = [p5(2) p6(2) p7(2) p8(2)]; y21 = [p51(2) p61(2) p71(2) p81(2)];
    # z2 = [p5(3) p6(3) p7(3) p8(3)]; z21 = [p51(3) p61(3) p71(3) p81(3)];
    x2 = [p5[1], p6[1], p7[1], p8[1]
    x21 = [p51[1], p61[1], p71[1], p81[1]]
    y2 = [p5[2], p6[2], p7[2] p8[2]]
    y21 = [p51[2], p61[2], p71[2], p81[2]]
    z2 = [p5[3], p6[3], p7[3], p8[3]
    z21 = [p51[3], p61[3], p71[3], p81[3]]


    # for i=0:2
    #     fill3(x1, y1, z1+i, 1);
    #     fill3(x11, y11, z11+i, 1)
    #     fill3(x2, y2+i*3, z2, 1);
    #     fill3(x21, y21+i*3, z21, 1);
    # end
    # xlabel('x'); ylabel('y'); zlabel('z');
    # alpha(0.3);
    # grid on;
    "^ Drawing, unnecessary for translation"

    # %find the location 
    # load('newinfo.mat')
    # %only use x coordinates between [-5,-3.75], y between [-6,6] and c [0 4]
    # %refrain from using boundary values for the time being
    # %A = [-4.7,0.5,3.2]; 
    # A= [x,y,z];
    # a = A (1); b=A(2); c=A(3);
    # x = section(:,1);
    # y = section(:,2);
    # z = section(:,3);
    A = [x, y, z]
    x = section(:, 1)
    y = section(:, 2)
    z = section(:, 3)

    # vec1=[];vec2=[];vec3=[];
    # for i=1:1:length(x)
    #     out1=[x{i,1}{1},x{i,1}{2}];
    #     vec1=[vec1; out1];
    #     out2=[y{i,1}{1},y{i,1}{2}];
    #     vec2=[vec2; out2];
    #     out3=[z{i,1}{1},z{i,1}{2}];
    #     vec3=[vec3; out3];
    # end
    vec1 = []
    vec2 = []
    vec3 = []
    for i in range(len(x)):
        out1 = []  # NOTE: Not sure how the outx vectors are calculated
        vec1.append(out1)
        out2 = []
        vec2.append(out2)
        out3 - []
        vec3.append(out3)


    #  %find the aisle
     
    # ind1 = [];
    # for j=1:length(vec1)
    #     if a > vec1(j,1) && a < vec1(j,2) %a >= vec1(j,1) && a <= vec1(j,2)
    #         ind1= [ind1,true];
    #     else
    #         ind1 = [ind1, false];
    #     end
    # end
    ind1 = []
    for j in range(len(vec1)):
        ind1.append(a > vec1(j, 1) && a < vec1(j, 2))

    # %ind(1)=[];
    # ind1=ind1';
    # log_vec1=find(ind1);
    # out_aisle=section(log_vec1(1),4);
    # outaisle=out_aisle{1}
    log_vec1 = ind1.nonzero()
    out_aisle = section(log_vec1[0], 3)
    outaisle = out_aisle[0]

    # %find the section 
    #  ind2 = [];
    # for j=1:length(vec2)
    #     if b > vec2(j,1) && b < vec2(j,2) %b >= vec2(j,1) && b <= vec2(j,2)
    #         ind2= [ind2,true];
    #     else
    #         ind2 = [ind2, false];
    #     end
    # end
    ind2 = []
    for j in range(len(vec2)):
        ind2.append(b > vec2(j,1) && b < vec2(j,2))

    # %ind(1)=[];
    # ind2=ind2';
    # log_vec2=find(ind2);
    # out_sec=section(log_vec2(1),5);
    # outsec=out_sec{1}
    log_vec2 = ind2.nonzero()
    out_sec = section(log_vec2[0], 4)
    outsec = out_sec[0]

    # %find the shelf
    # ind3 = [];
    # for j=1:length(vec3)
    #     if c > vec3(j,1) && c < vec3(j,2) %c >= vec3(j,1) && c <= vec3(j,2)
    #         ind3= [ind3,true];
    #     else
    #         ind3 = [ind3, false];
    #     end
    # end
    ind3 = []
    for j in range(len(vec3)):
        ind3.append(c > vec3(j,1) && c < vec3(j,2))

    # %ind(1)=[];
    # ind3=ind3';
    # log_vec3=find(ind3);
    # out_shelf=section(log_vec3(1),6);
    # outshelf=out_shelf{1}
    log_vec3 = ind3.nonzero()
    out_shelf = section(log_vec3[0], 5)
    outshelf = out_shelf[0]

    # name = [outaisle,outsec,outshelf];
    # out=name;
    # scatter3(A(1),A(2),A(3));
    # ts = text(A(1),A(2),A(3),name);
    # end
    name = [outaisle, outsec, outshelf]
    return name

