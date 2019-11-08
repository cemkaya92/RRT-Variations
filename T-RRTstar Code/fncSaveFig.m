%%function []=plottingScript



%%
% matlab plots



% fncSaveFig(sw_figure,pathFigs,'input_signals',9,6.5,2.5)



%%
function [] = fncSaveFig(flgPrintPlot,pathFigs,nameFigure,fontsizeN,varargin)
% flgPrintPlot: whether to save the figures as "grayscaled" and "colored" pdf and eps figures
% pathFigs:  path of the folder the pdf and eps figures are saved
% nameFigure: base name of the figures. grayscale figures have "BW", and colored figures have "CL" addition to the name.
% fontsizeN: size of the fonts (axis labels, legends, title etc) in the figures
% varargin{1} = plot width:  width of the figure in inches on paper
% varargin{2} = plot height: height of the figure in inches on paper

% sw_figure: =0 only plot, =1 plot and save
sw_figure = 1; % 1=Print and Save

if flgPrintPlot==1    
    saveas(gcf,[pathFigs,nameFigure,'.fig'])
    if nargin > 4
%         exportfig(gcf,[pathFigs,nameFigure,'BW.eps'],'width',varargin{1},'height',varargin{2},'fontmode','fixed','fontsize',fontsizeN,'Color','gray','LineWidthMax',1,'LockAxes',0)
        exportfig(gcf,[pathFigs,nameFigure,'CL.eps'],'width',varargin{1},'height',varargin{2},'fontmode','fixed','fontsize',fontsizeN,'Color','cmyk','LockAxes',0)
    else
%         exportfig(gcf,[pathFigs,nameFigure,'BW.eps'],'width',5,'height',5,'fontmode','fixed','fontsize',fontsizeN,'Color','gray','LineWidthMax',1,'LockAxes',0)
        exportfig(gcf,[pathFigs,nameFigure,'CL.eps'],'width',5,'height',5,'fontmode','fixed','fontsize',fontsizeN,'Color','cmyk','LockAxes',0)
    end
    
%     [result,msg] = eps2pdf([pathFigs,nameFigure,'BW.eps'],'C:\Program Files\gs\gs9.20\bin\gswin64c.exe')
    [result,msg] = eps2pdf([pathFigs,nameFigure,'CL.eps'],'C:\Program Files\gs\gs9.20\bin\gswin64c.exe')
    close
end
%%return
end

