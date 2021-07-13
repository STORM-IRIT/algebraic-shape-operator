import matplotlib.pyplot as plt

values_method = ['Barycenter','OJets','PCAPlane','PSS','WJets','VCM','APSS','ASO']
values_n_int  = [10000,25000,50000,75000,100000,250000,500000,750000,1000000]
values_n_str  = ['0010000','0025000','0050000','0075000','0100000','0250000','0500000','0750000','1000000']
surface = "goursat"

class Style:
    def __init__(self, c, ls):
        self.c = c
        self.ls = ls

styles = {
    'Barycenter': Style('#53ecec', '-'),
    'PCAPlane':   Style('#5353ec', '-'),
    'APSS':       Style('#ecec53', '-'),
    'ASO':        Style('#ec5353', '-'),
    'OJets':      Style('#a053ec', '-'),
    'WJets':      Style('#a07c7c', '-'),
    'PSS':        Style('#f6aaf6', '-'),
    'VCM':        Style('#006600', '-')
}

class Statistics:
    def __init__(self, line_string):
        self.from_line_string(line_string)

    def from_line_string(self, line_string):
        strings = line_string.split()
        assert(len(strings) == 7)
        self.min     = float(strings[0])
        self.c25     = float(strings[1])
        self.c50     = float(strings[2])
        self.c75     = float(strings[3])
        self.max     = float(strings[4])
        self.mean    = float(strings[5])
        self.std_var = float(strings[6])

xmin = 10000
xmax = 1000000
ymin = 0.00041398165073971863 
ymax = 0.2746661295949335

fig, ax = plt.subplots()

for method in values_method:
    x = []
    y = []

    for idx_n in range(len(values_n_str)):
        n_int = values_n_int[idx_n]
        n_str = values_n_str[idx_n]

        filename = 'stats/' + surface + '_' + n_str + '_' + method + '.txt'
        file = open(filename, 'r')
        lines = file.readlines()
        assert(len(lines) == 8)

        stat_N = Statistics(lines[3])

        x.append(n_int)
        y.append(stat_N.mean)

    ax.plot(x, y, c=styles[method].c, ls=styles[method].ls, label=method, linewidth = 2.5)
    
plt.xlabel('N')
plt.ylabel('normal vector deviation')
plt.grid(color = 'gray', linestyle = '--', linewidth = 1)
ax.set_xlim([xmin,xmax])
plt.yscale('log')
plt.legend()
plt.savefig('results/error_n.pdf')

plt.show()
