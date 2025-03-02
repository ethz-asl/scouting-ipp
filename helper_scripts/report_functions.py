import os
import numpy as np
from pylatex.package import Package
from pylatex.utils import italic, NoEscape
from pylatex.base_classes import Environment, CommandBase, Arguments
from pylatex import Document, Section, Subsection, Tabular, Math, TikZ, Axis, Plot, Figure, Matrix, Alignat, Command, \
    SubFigure, UnsafeCommand, Center, TikZ, TikZNode, TikZDraw, TikZCoordinate, TikZUserPath, TikZOptions


class Square(CommandBase):
    """
    A class representing a custom LaTeX command.

    This class represents a custom LaTeX command named
    ``exampleCommand``.
    """

    _latex_name = 'square'
    packages = [Package('color'), Package('tikz')]


def generate_report(dest, map_file, vis_file, path_image, cost, labels, label_count, relative_occurance, obstacle_ids, obstacle_pixel, obstacle_percentage, colors, vis_colors):
    geometry_options = {"tmargin": "1cm", "lmargin": "2cm", "bmargin": "2cm"}
    doc = Document(geometry_options=geometry_options)

    #create colors
    new_comm = UnsafeCommand('newcommand', '\square', options=1,
                             extra_arguments=r'{\tikz{\filldraw[draw=#1,fill=#1] (0,0) rectangle (1.5em,1.5em);}}')
    doc.append(new_comm)
    for i, color in enumerate(colors):
        color_str = str(color[2]/255.0) + ", " + str(color[1]/255.0) + ", " + str(color[0]/255.0)
        doc.add_color(name=str(labels[i]) + "color", model="rgb", description=color_str)
    for i, color in enumerate(vis_colors):
        color_str = str(color[2] / 255.0) + ", " + str(color[1] / 255.0) + ", " + str(color[0] / 255.0)
        doc.add_color(name=str(labels[i]) + "viscolor", model="rgb", description=color_str)

    doc.preamble.append(Command('title', map_file.split(os.sep)[-1]))
    doc.preamble.append(Command('date', NoEscape(r'\vspace{-3em}')))
    doc.append(NoEscape(r'\maketitle'))
    with doc.create(Figure(position='h!')) as maps:
        doc.append(NoEscape('{'))
        doc.append(Command('centering'))
        with doc.create(SubFigure(
                position='b',
                width=NoEscape(r'0.45\linewidth'))) as map:
            doc.append(NoEscape('{'))
            doc.append(Command('centering'))
            map.add_image(map_file,
                                  width=NoEscape(r'0.9\linewidth'))
            map.add_caption('Original Map')
            doc.append(NoEscape('}'))
        with doc.create(SubFigure(
                position='b',
                width=NoEscape(r'0.45\linewidth'))) as map_vis:
            doc.append(NoEscape('{'))
            doc.append(Command('centering'))
            map_vis.add_image(vis_file,
                                   width=NoEscape(r'0.9\linewidth'))
            map_vis.add_caption('Map Visualization')
            doc.append(NoEscape('}'))
        maps.add_caption("Input Map")
        doc.append(NoEscape('}'))

    with doc.create(Section('Analysis ')):
        with doc.create(Subsection('Optimal Path')):
            doc.append("Optimal path cost: " + str(cost))
            if os.path.isfile(path_image):
                with doc.create(Figure(position='h!')) as map_path:
                    map_path.add_image(path_image,
                                  width=NoEscape(r'0.4\linewidth'))
                    map_path.add_caption('Map with best Path')
            else:
                doc.append("\n\nRRT* planner was not able to find solution!!!")
        with doc.create(Subsection('Label Statistic')):
            obstacle_string = "Obstacle ids: ["
            separation_string = ""
            for obstacle_id in obstacle_ids:
                obstacle_string += separation_string
                obstacle_string += str(obstacle_id)
                separation_string = ", "
            obstacle_string += "]"
            doc.append(obstacle_string)

            for j in range(int(np.ceil((len(labels)+1)/15.0))):
                if j == 0:
                    table_sep = "c|"
                    first_line = ["O"]
                    second_line = ["white"]
                    second_line_vis = ["white"]
                    third_line = [obstacle_pixel]
                    fourth_line = [str(np.round(obstacle_percentage * 100, 1)) + "%"]
                    start_idx = 0
                    end_idx = min(13, len(labels))
                else:
                    table_sep = ""
                    first_line = []
                    second_line = []
                    second_line_vis = []
                    third_line = []
                    fourth_line = []
                    start_idx = end_idx + 1
                    end_idx = min(start_idx + 14, len(labels))
                for i, label in enumerate(labels[start_idx:end_idx]):
                    table_sep += "c|"
                    second_line.append(str(label) + "color")
                    second_line_vis.append(str(label) + "viscolor")
                    first_line.append(label)
                    third_line.append(label_count[label])
                    fourth_line.append(str(np.round(relative_occurance[label] * 100, 1)) + "%")
                with doc.create(Center()):
                    with doc.create(Tabular(table_sep)) as table:
                        table.add_row(first_line)
                        table.add_row(second_line, mapper=Square)
                        table.add_row(second_line_vis, mapper=Square)
                        table.add_hline()
                        table.add_row(third_line)
                        table.add_row(fourth_line)
                        #table.add_empty_row()


    with doc.create(TikZ()):
        pass
    doc.generate_pdf(dest, clean=True, clean_tex=True)