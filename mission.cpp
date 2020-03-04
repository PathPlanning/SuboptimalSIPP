#include"mission.h"

Mission::Mission()
{
    m_pSearch = nullptr;
    m_pLogger = nullptr;
}

Mission::~Mission()
{
    delete m_pSearch;
    delete m_pLogger;
}

void Mission::setFileNames(const char *taskName, const char *mapName, const char *configName, const char *obstaclesName)
{
    this->mapName = mapName;
    this->taskName = taskName;
    this->configName = configName;
    this->obstaclesName = obstaclesName;
}

bool Mission::getMap()
{
    return m_map.getMap(mapName);
}

bool Mission::getTask()
{
    return (m_task.getTask(taskName) && m_task.validateTask(m_map));
}

bool Mission::getConfig()
{
    return m_config.getConfig(configName);
}

bool Mission::getObstacles()
{
    if(obstaclesName)
        return m_obstacles.getObstacles(obstaclesName);
    else
        return false;
}

void Mission::createSearch()
{
    if(m_pSearch)
    {
        delete m_pSearch;
        delete m_pLogger;
    }
    if(m_config.algtype == 1)
    {
        m_config.use_focal = false;
        m_config.use_likhachev = true;
        m_config.h_weight = m_config.weight;
        m_config.focal_weight = 1.0;
    }
    else if(m_config.algtype == 2)
    {
        m_config.use_focal = false;
        m_config.use_likhachev = false;
        m_config.h_weight = m_config.weight;
        m_config.focal_weight = 1.0;
    }
    else if(m_config.algtype == 3)
    {
        m_config.use_focal = true;
        m_config.use_likhachev = false;
        m_config.h_weight = 1.0;
        m_config.focal_weight = m_config.weight;
    }
    m_pSearch = new AA_SIPP(m_config);
}

void Mission::createLog()
{
    if(m_config.loglevel != CN_LOGLVL_NO)
    {
        m_pLogger = new XmlLogger(m_config.loglevel);
        m_pLogger->createLog(taskName);
    }
}

void Mission::startSearch()
{
    //std::cout<<"SEARCH STARTED\n";

    sr = m_pSearch->startSearch(m_map, m_task, m_obstacles);
}

void Mission::printSearchResultsToConsole()
{
    std::ofstream out("den520d_no_rot_k5_all.txt", std::ios_base::app);
    out<<sr.runtime<<" "<<sr.makespan<<" "<<sr.pathInfo[0].expanded<<" "<<sr.pathInfo[0].generated<<" "<<sr.pathInfo[0].reexpanded<<" "<<sr.pathInfo[0].reopened<<" "<<sr.pathInfo[0].sections.size()<<"\n";
    out.close();
    std::cout<<sr.runtime<<" "<<sr.makespan<<" "<<sr.pathInfo[0].expanded<<" "<<sr.pathInfo[0].generated<<" "<<sr.pathInfo[0].reexpanded<<" "<<sr.pathInfo[0].reopened<<" "<<sr.pathInfo[0].sections.size()<<"\n";
    //std::cout<<"Results:\nTask solved: "<<bool(sr.agentsSolved/sr.agents)<<"\nTries: "<<sr.tries<<"\nRuntime: "<<sr.runtime<<"\nAgents solved: "<<sr.agentsSolved<<" ("<<(float)sr.agentsSolved*100/sr.agents<<"%)\nFlowtime: "<<sr.flowtime<<"\nMakespan: "<<sr.makespan<<"\n";
}

void Mission::saveSearchResultsToLog()
{
    if(m_config.loglevel == CN_LOGLVL_NO)
        return;
    std::cout<<"LOG STARTED\n";
    m_pLogger->writeToLogInput(taskName, mapName, configName, obstaclesName);
    m_pLogger->writeToLogSummary(sr);
    if(sr.pathfound)
    {
        m_pLogger->writeToLogMap(m_map, sr);
        m_pLogger->writeToLogPath(sr, m_task, m_config);
    }
    m_pLogger->saveLog();
    std::cout<<"LOG SAVED\n";
}

