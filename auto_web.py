from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time

PATH = "/home/jing/Documents/Softwares/chromedriver"
driver = webdriver.Chrome(PATH)
driver.get("https://www.nike.com.hk/product/CT0978-005/detail.htm?pdpRecommend=false&preSkuCode=")
soldOutFlag = "bg888"
time.sleep(1)

soldOutClass = driver.find_element_by_xpath("//div[@class='if-you-like-sold-out iyl-button' and style='']")
if soldOutClass:
    print("This item is sold out")

driver.quit()
